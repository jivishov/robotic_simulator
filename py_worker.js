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

  mod.line_to = new Sk.builtin.func(function (x, y, steps) {
    const X = Number(Sk.ffi.remapToJs(x));
    const Y = Number(Sk.ffi.remapToJs(y));
    const st = (steps === undefined) ? 50 : Number(Sk.ffi.remapToJs(steps));
    const id = nextId();
    return Sk.misceval.promiseToSuspension(makeAwaitableCommand({ id, type: "LINE_TO", x: X, y: Y, steps: st }));
  });

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
  if (Sk.builtinFiles === undefined || Sk.builtinFiles["files"][x] === undefined) {
    throw new Error("File not found: '" + x + "'");
  }
  return Sk.builtinFiles["files"][x];
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

    Sk.configure({
      output: (text) => postPrint(text),
      read: builtinRead,
      __future__: Sk.python3,
      execLimit: 2_500_000,
    });

    // Pre-populate Sk.sysmodules with 'arm' and 'sim' modules
    // This directly injects modules into Python's module cache
    function createModule(name, apiObj) {
      var mod = new Sk.builtin.module();
      mod.$d = {};
      mod.$d.__name__ = new Sk.builtin.str(name);
      for (var k in apiObj) {
        if (apiObj.hasOwnProperty(k)) {
          mod.$d[k] = apiObj[k];
        }
      }
      return mod;
    }
    Sk.sysmodules = Sk.sysmodules || new Sk.builtin.dict([]);
    Sk.sysmodules.mp$ass_subscript(new Sk.builtin.str("arm"), createModule("arm", armModule()));
    Sk.sysmodules.mp$ass_subscript(new Sk.builtin.str("sim"), createModule("sim", simModule()));

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
