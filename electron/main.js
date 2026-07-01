const { app, BrowserWindow, ipcMain } = require("electron");
const path = require("path");

// หมายเหตุ: การเชื่อมต่อ Serial Port จริงกับบอร์ด ESP32 ใช้ไลบรารี "serialport"
// ซึ่งเป็น native module (ต้อง build แยกตามระบบปฏิบัติการ) — ผู้ใช้ต้องติดตั้งเพิ่มเอง:
//   npm install serialport
// แล้วค่อย uncomment โค้ดด้านล่างเพื่อเปิดใช้งานจริง
// const { SerialPort } = require("serialport");

let mainWindow;
let activePort = null;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1440,
    height: 900,
    minWidth: 1100,
    minHeight: 700,
    backgroundColor: "#0a0e1a",
    autoHideMenuBar: true, // ซ่อนแถบเมนูมาตรฐานของ Electron เพราะ UI มีแถบของตัวเองแล้ว
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
    mainWindow.loadFile(path.join(__dirname, "..", "dist", "index.html"));
  }
}

app.whenReady().then(() => {
  createWindow();
  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});

// ---- IPC: serial port (โครงไว้ให้ ต่อกับไลบรารี serialport จริงตามต้องการ) ----
ipcMain.handle("serial:list", async () => {
  // ตัวอย่าง (เมื่อติดตั้ง serialport แล้ว):
  // const ports = await SerialPort.list();
  // return ports.map((p) => p.path);
  return ["COM3", "COM4", "COM5"]; // รายการจำลอง — แทนที่ด้วยของจริงเมื่อพร้อม
});

ipcMain.handle("serial:connect", async (event, portName) => {
  // activePort = new SerialPort({ path: portName, baudRate: 115200 });
  activePort = portName;
  return { ok: true, port: portName };
});

ipcMain.handle("serial:disconnect", async () => {
  // if (activePort) activePort.close();
  activePort = null;
  return { ok: true };
});

ipcMain.handle("serial:send", async (event, data) => {
  // if (activePort) activePort.write(JSON.stringify(data) + "\n");
  return { ok: true };
});
