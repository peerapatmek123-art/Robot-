const { app, BrowserWindow, ipcMain } = require("electron");
const path = require("path");

// ---------------------------------------------------------------------------
// การเชื่อมต่อ Serial Port กับบอร์ด ESP32
// ---------------------------------------------------------------------------
// ใช้ไลบรารี "serialport" (native module) ซึ่งถูกเพิ่มไว้ใน package.json แล้ว
// รันคำสั่งนี้ครั้งเดียวก่อนใช้งาน:
//     npm install
// ถ้ายังไม่เคย build native module ให้ตรงกับ Electron ให้รันเพิ่ม:
//     npx electron-builder install-app-deps
//
// ถ้ายังไม่ได้ติดตั้ง serialport (เช่นตอน dev เร็วๆ โดยไม่มีฮาร์ดแวร์) แอปจะ
// fallback ไปใช้โหมดจำลอง (mock) โดยอัตโนมัติ ไม่ทำให้แอปพัง
let SerialPort = null;
let ReadlineParser = null;
try {
  ({ SerialPort } = require("serialport"));
  ({ ReadlineParser } = require("@serialport/parser-readline"));
} catch (err) {
  console.warn(
    "[serial] ไม่พบไลบรารี 'serialport' — ทำงานในโหมดจำลอง (mock). " +
      "รัน `npm install` เพื่อเปิดใช้งานการเชื่อมต่อฮาร์ดแวร์จริง"
  );
}

// ---------------------------------------------------------------------------
// โปรโตคอลการสื่อสาร (App ⇄ ESP32) — JSON บรรทัดเดียว คั่นด้วย "\n"
// ---------------------------------------------------------------------------
// ทิศทาง App → ESP32 (สั่งมุมข้อต่อ):
//   {"type":"joints","j1":0,"j2":10,"j3":15,"j4":30,"j5":20,"j6":0}\n
//   - j1..j6 หน่วยเป็นองศา (float), ช่วงตามการออกแบบกลไกจริง (ปกติ -180..180)
//   - ESP32 ควรตอบกลับทันทีด้วย ack เพื่อให้แอปมั่นใจว่าได้รับคำสั่งแล้ว:
//     {"type":"ack","ok":true}\n
//
// ทิศทาง ESP32 → App (ไม่บังคับ แต่แนะนำให้มี เพื่อโชว์ในแท็บ System Status):
//   {"type":"telemetry","voltage":12.0,"tempC":36.2,"estop":false}\n
//
// Baud rate: 115200 (ตรงกับที่ตั้งไว้ในแอปและ UI แท็บ System Status)
const BAUD_RATE = 115200;

let mainWindow;
let port = null; // instance ของ SerialPort เมื่อเชื่อมต่อจริง
let parser = null;
let mockConnectedPortName = null; // ใช้เฉพาะตอนไม่มีไลบรารี serialport

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

    mainWindow.webContents.openDevTools();

});

app.on("window-all-closed", () => {
  if (process.platform !== "darwin") app.quit();
});

app.on("before-quit", () => {
  if (port?.isOpen) port.close();
});

// ---- IPC: serial port -----------------------------------------------------
ipcMain.handle("serial:list", async () => {
  if (SerialPort) {
    try {
      const ports = await SerialPort.list();
      return ports.map((p) => p.path);
    } catch (err) {
      console.error("[serial] list ports failed:", err.message);
      return [];
    }
  }
  // โหมดจำลอง — ไม่มีฮาร์ดแวร์จริง
  return ["COM3", "COM4", "COM5"];
});

ipcMain.handle("serial:connect", async (event, portName) => {
  if (!SerialPort) {
    // โหมดจำลอง: ยืนยันว่า "เชื่อมต่อ" แค่ตั้งชื่อพอร์ตไว้เฉยๆ
    mockConnectedPortName = portName;
    return { ok: true, port: portName, mock: true };
  }

  return new Promise((resolve) => {
    try {
      if (port?.isOpen) port.close();

      port = new SerialPort({ path: portName, baudRate: BAUD_RATE }, (err) => {
        if (err) {
          console.error("[serial] connect failed:", err.message);
          port = null;
          resolve({ ok: false, error: err.message });
          return;
        }
      });

      port.once("open", () => {
        // ตั้ง parser อ่านทีละบรรทัด เพื่อรับ ack / telemetry จาก ESP32
        parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));
        parser.on("data", (line) => {
          try {
            const msg = JSON.parse(line);
            mainWindow?.webContents.send("serial:data", msg);
          } catch {
            // ไม่ใช่ JSON ที่ถูกต้อง — ข้ามไป (เช่น log debug จากบอร์ด)
          }
        });

        port.on("close", () => {
          mainWindow?.webContents.send("serial:data", { type: "disconnected" });
        });

        resolve({ ok: true, port: portName });
      });

      port.once("error", (err) => {
        console.error("[serial] port error:", err.message);
      });
    } catch (err) {
      resolve({ ok: false, error: err.message });
    }
  });
});

ipcMain.handle("serial:disconnect", async () => {
  if (!SerialPort) {
    mockConnectedPortName = null;
    return { ok: true };
  }
  if (port?.isOpen) {
    await new Promise((resolve) => port.close(resolve));
  }
  port = null;
  parser = null;
  return { ok: true };
});

// ตรวจสอบสถานะการเชื่อมต่อ "จริง" — อิงจาก port.isOpen ไม่ใช่ค่าที่เคยตั้งไว้เฉยๆ
ipcMain.handle("serial:status", async () => {
  if (!SerialPort) {
    return { connected: mockConnectedPortName !== null, port: mockConnectedPortName, mock: true };
  }
  return { connected: !!port?.isOpen, port: port?.path ?? null };
});

ipcMain.handle("serial:send", async (event, data) => {
  // โปรโตคอล JSON ที่ส่งไป ESP32 (5-DOF):
  // {"type":"joints","j1":<deg>,"j2":<deg>,"j3":<deg>,"j4":<deg>,"j5":<0-100%>}\n
  // j1 = ฐานหมุน (N20 AB Encoder, -180..180 deg)
  // j2,j3,j4 = ขึ้น-ลง (deg)
  // j5 = Gripper เปิด/ปิด symmetric (0=ปิดสนิท, 100=เปิดสุด, หน่วย %)
  const payload = JSON.stringify({ type: "joints", ...data }) + "\n";

  if (!SerialPort) {
    console.log("[serial:mock] would send:", payload.trim());
    return { ok: true, mock: true };
  }
  if (!port?.isOpen) {
    return { ok: false, error: "ยังไม่ได้เชื่อมต่อพอร์ต" };
  }
  return new Promise((resolve) => {
    port.write(payload, (err) => {
      if (err) resolve({ ok: false, error: err.message });
      else resolve({ ok: true });
    });
  });
});
