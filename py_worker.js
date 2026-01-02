// py_worker.js (classic worker)
// Loads Skulpt from CDN and runs user Python in a sandboxed, interruptible environment.

importScripts("https://cdn.jsdelivr.net/npm/skulpt@1.2.0/dist/skulpt.min.js");
importScripts("https://cdn.jsdelivr.net/npm/skulpt@1.2.0/dist/skulpt-stdlib.js");

let runToken = 0;
let pending = new Map(); // cmdId -> {resolve,reject}
let stopped = false;

function postPrint(s) {
  self.postMessage({ type: "print", text: String(s) });
}
function postError(s) {
  self.postMessage({ type: "error", text: String(s) });
}

function makeAwaitableCommand(cmd) {
  const id = cmd.id;
  self.postMessage({ type: "enqueue_cmd", cmd });
  return new Promise((resolve, reject) => {
    pending.set(String(id), { resolve, reject });
  });
}

// Builtin modules: arm, sim
function armModule() {
  const mod = {};

  mod.set_speed = new Sk.builtin.func(function (v) {
    const val = Sk.ffi.remapToJs(v);
    self.postMessage({ type: "set_speed", value: Number(val) });
    return Sk.builtin.none.none$;
  });

  mod.home = new Sk.builtin.func(function () {
    const id = nextId();
    return Sk.misceval.promiseToSuspension(makeAwaitableCommand({ id, type: "HOME" }));
  });

  mod.set_joint_angles = new Sk.builtin.func(function (lst) {
    const arr = Sk.ffi.remapToJs(lst);
    if (!Array.isArray(arr) || arr.length < 2) throw new Sk.builtin.ValueError("Expected [deg1, deg2]");
    const id = nextId();
    return Sk.misceval.promiseToSuspension(makeAwaitableCommand({ id, type: "SET_JOINTS", deg: [Number(arr[0]), Number(arr[1])] }));
  });

  mod.move_to = new Sk.builtin.func(function (x, y) {
    const X = Number(Sk.ffi.remapToJs(x));
    const Y = Number(Sk.ffi.remapToJs(y));
    const id = nextId();
    return Sk.misceval.promiseToSuspension(makeAwaitableCommand({ id, type: "MOVE_TO", x: X, y: Y }));
  });

  mod.line_to = new Sk.builtin.func(function (kwa, x, y, steps) {
    // Handle keyword arguments - kwa is an array of [key, value, key, value, ...]
    Sk.abstr.checkArgsLen("line_to", arguments, 3, 4);
    var kwdict = {};
    if (kwa) {
      for (var i = 0; i < kwa.length; i += 2) {
        kwdict[kwa[i]] = kwa[i + 1];
      }
    }
    const X = Number(Sk.ffi.remapToJs(x));
    const Y = Number(Sk.ffi.remapToJs(y));
    // steps can come from positional arg or keyword arg
    var stepsVal = steps !== undefined ? steps : kwdict["steps"];
    const st = (stepsVal === undefined) ? 50 : Number(Sk.ffi.remapToJs(stepsVal));
    const id = nextId();
    return Sk.misceval.promiseToSuspension(makeAwaitableCommand({ id, type: "LINE_TO", x: X, y: Y, steps: st }));
  });
  mod.line_to.co_kwargs = true;

  return mod;
}

function simModule() {
  const mod = {};

  mod.sleep = new Sk.builtin.func(function (sec) {
    const seconds = Math.max(0, Number(Sk.ffi.remapToJs(sec)));
    const id = nextId();
    return Sk.misceval.promiseToSuspension(makeAwaitableCommand({ id, type: "SLEEP", seconds }));
  });

  mod.reset_path = new Sk.builtin.func(function () {
    self.postMessage({ type: "reset_path" });
    return Sk.builtin.none.none$;
  });

  return mod;
}

function builtinRead(x) {
  if (Sk.builtinFiles && Sk.builtinFiles["files"] && Sk.builtinFiles["files"][x] !== undefined) {
    return Sk.builtinFiles["files"][x];
  }
  throw new Error("File not found: '" + x + "'");
}

let _nextId = 1;
function nextId() { return _nextId++; }

self.onmessage = async (ev) => {
  const msg = ev.data;
  if (!msg || !msg.type) return;

  if (msg.type === "stop") {
    stopped = true;
    runToken++;
    for (const [id, pr] of pending.entries()) {
      pr.reject(new Error("stopped"));
    }
    pending.clear();
    return;
  }

  if (msg.type === "cmd_done") {
    const id = String(msg.id);
    const pr = pending.get(id);
    if (pr) {
      pending.delete(id);
      if (msg.result && msg.result.ok === false) pr.reject(new Error(msg.result.error || "command failed"));
      else pr.resolve(msg.result);
    }
    return;
  }

  if (msg.type === "sim_reset_done") {
    _nextId = 1;
    return;
  }

  if (msg.type === "run") {
    stopped = false;
    const myToken = ++runToken;
    pending.clear();
    _nextId = 1;

    const code = String(msg.code || "");
    self.postMessage({ type: "reset_path" });

    // Expose functions on self so they're accessible from eval'd module code
    self._armModule = armModule;
    self._simModule = simModule;

    // Add our custom modules to Sk.builtinFiles BEFORE configure
    // Skulpt looks for modules at src/lib/MODULE.js
    Sk.builtinFiles = Sk.builtinFiles || {};
    Sk.builtinFiles["files"] = Sk.builtinFiles["files"] || {};
    Sk.builtinFiles["files"]["src/lib/arm.js"] =
      "var $builtinmodule = function(name) { return self._armModule(); };";
    Sk.builtinFiles["files"]["src/lib/sim.js"] =
      "var $builtinmodule = function(name) { return self._simModule(); };";

    Sk.configure({
      output: (text) => postPrint(text),
      read: builtinRead,
      __future__: Sk.python3,
      execLimit: 2_500_000,
    });

    try {
      const p = Sk.misceval.asyncToPromise(() => Sk.importMainWithBody("<stdin>", false, code, true));
      await p;
      if (myToken === runToken && !stopped) postPrint("[done]");
    } catch (err) {
      if (myToken !== runToken) return;
      postError(String(err));
    }
  }
};
