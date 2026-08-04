const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("electronAPI", {
  listPorts: () => ipcRenderer.invoke("serial:list"),
  connectPort: (portName) => ipcRenderer.invoke("serial:connect", portName),
  disconnectPort: () => ipcRenderer.invoke("serial:disconnect"),
  getStatus: () => ipcRenderer.invoke("serial:status"),
  sendJointAngles: (data) => ipcRenderer.invoke("serial:send", data),
  // RoboticArmControl.jsx เรียก sendSerial(jsonStr) ด้วย JSON string ที่มี
  // "type":"joints" ติดมาด้วยแล้ว — main.js (serial:send) รับเป็น object และ
  // เติม type:"joints" ให้เองอยู่แล้ว จึงแค่ parse string กลับเป็น object ก่อนส่งต่อ
  sendSerial: (jsonStr) => {
    let data;
    try {
      data = JSON.parse(jsonStr);
    } catch (err) {
      return Promise.resolve({ ok: false, error: "JSON ไม่ถูกต้อง: " + err.message });
    }
    return ipcRenderer.invoke("serial:send", data);
  },
  waitUntilDone: () => ipcRenderer.invoke("serial:waitUntilDone"),
  // รับข้อความ (ack / telemetry) ที่ ESP32 ส่งกลับมาแบบ real-time
  // คืนค่าฟังก์ชันสำหรับ unsubscribe เมื่อเลิกใช้
  onSerialData: (callback) => {
    const handler = (event, msg) => callback(msg);
    ipcRenderer.on("serial:data", handler);
    return () => ipcRenderer.removeListener("serial:data", handler);
  },
});
