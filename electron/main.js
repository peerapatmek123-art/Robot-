const { app, BrowserWindow, ipcMain } = require("electron");
const path = require("path");

let SerialPort = null;
let ReadlineParser = null;

try {
  ({ SerialPort } = require("serialport"));
  ({ ReadlineParser } = require("@serialport/parser-readline"));
} catch (err) {
  console.warn(
    "[serial] ไม่พบไลบรารี serialport (Mock Mode)"
  );
}

const BAUD_RATE = 115200;

let mainWindow;
let port = null;
let parser = null;

let doneResolver = null;
let mockConnectedPortName = null;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 1100,
    minHeight: 700,
    backgroundColor: "#0a0e1a",
    autoHideMenuBar: true,
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
    },
  });

  const devUrl = process.env.VITE_DEV_SERVER_URL;

  if (devUrl) {
    mainWindow.loadURL(devUrl);
  } else {
    mainWindow.loadFile(
      path.join(__dirname, "..", "dist", "index.html")
    );
  }
}

app.whenReady().then(() => {
  createWindow();

  mainWindow.webContents.openDevTools();

  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0)
      createWindow();
  });
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});

app.on("before-quit", () => {
  if (port?.isOpen) port.close();
});


// ----------------------------------------------------
// List Ports
// ----------------------------------------------------

ipcMain.handle("serial:list", async () => {

  if (!SerialPort)
    return ["COM3", "COM4", "COM5"];

  try {

    const ports = await SerialPort.list();

    return ports.map((p) => p.path);

  } catch (err) {

    console.error(err);

    return [];

  }

});


// ----------------------------------------------------
// Connect
// ----------------------------------------------------

ipcMain.handle("serial:connect", async (event, portName) => {

  if (!SerialPort) {

    mockConnectedPortName = portName;

    return {
      ok: true,
      mock: true,
      port: portName,
    };

  }

  return new Promise((resolve) => {

    try {

      if (port?.isOpen)
        port.close();

      port = new SerialPort({

        path: portName,
        baudRate: BAUD_RATE,

      });

      port.once("open", () => {

        console.log("Connected :", portName);

        parser = port.pipe(
          new ReadlineParser({
            delimiter: "\n",
          })
        );

        parser.on("data", (line) => {

          line = line.trim();

          console.log("ESP32 >", line);

          if (line === "DONE") {

            if (doneResolver) {

              doneResolver(true);

              doneResolver = null;

            }

            return;

          }

          try {

            const msg = JSON.parse(line);

            mainWindow?.webContents.send(
              "serial:data",
              msg
            );

          } catch {

            // ไม่ใช่ JSON

          }

        });

        port.on("close", () => {

          console.log("Serial Closed");

          mainWindow?.webContents.send(
            "serial:data",
            {
              type: "disconnected",
            }
          );

        });

        resolve({

          ok: true,

          port: portName,

        });

      });

      port.once("error", (err) => {

        console.error(err);
    
        resolve({
    
            ok: false,
    
            error: err.message,
    
        });
    
    });

    } catch (err) {

      resolve({

        ok: false,

        error: err.message,

      });

    }

  });

});
// ----------------------------------------------------
// Disconnect
// ----------------------------------------------------

ipcMain.handle("serial:disconnect", async () => {

  if (!SerialPort) {

    mockConnectedPortName = null;

    return {
      ok: true,
    };

  }

  if (port?.isOpen) {

    await new Promise((resolve) => port.close(resolve));

  }

  port = null;
  parser = null;

  return {
    ok: true,
  };

});


// ----------------------------------------------------
// Status
// ----------------------------------------------------

ipcMain.handle("serial:status", async () => {

  if (!SerialPort) {

    return {

      connected: mockConnectedPortName !== null,

      port: mockConnectedPortName,

      mock: true,

    };

  }

  return {

    connected: !!port?.isOpen,

    port: port?.path ?? null,

  };

});


// ----------------------------------------------------
// Wait Until DONE
// ----------------------------------------------------

ipcMain.handle("serial:waitUntilDone", () => {

  return new Promise((resolve) => {

    doneResolver = resolve;

  });

});


// ----------------------------------------------------
// Send Joint Angles
// ----------------------------------------------------

ipcMain.handle("serial:send", async (event, data) => {

  const payload =
    JSON.stringify({
      type: "joints",
      ...data,
    }) + "\n";

  if (!SerialPort) {

    console.log("[MOCK SEND]", payload.trim());

    return {

      ok: true,

      mock: true,

    };

  }

  if (!port?.isOpen) {

    return {

      ok: false,

      error: "ยังไม่ได้เชื่อมต่อพอร์ต",

    };

  }

  return new Promise((resolve) => {

    port.write(payload, (err) => {

      if (err) {

        resolve({

          ok: false,

          error: err.message,

        });

      } else {

        resolve({

          ok: true,

        });

      }

    });

  });

});
