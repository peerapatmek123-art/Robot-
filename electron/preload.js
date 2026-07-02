const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("electronAPI", {
  listPorts: () => ipcRenderer.invoke("serial:list"),
  connectPort: (portName) => ipcRenderer.invoke("serial:connect", portName),
  disconnectPort: () => ipcRenderer.invoke("serial:disconnect"),
  getStatus: () => ipcRenderer.invoke("serial:status"),
  sendJointAngles: (data) => ipcRenderer.invoke("serial:send", data),
  // รับข้อความ (ack / telemetry) ที่ ESP32 ส่งกลับมาแบบ real-time
  // คืนค่าฟังก์ชันสำหรับ unsubscribe เมื่อเลิกใช้
  onSerialData: (callback) => {
    const handler = (event, msg) => callback(msg);
    ipcRenderer.on("serial:data", handler);
    return () => ipcRenderer.removeListener("serial:data", handler);
  },
});
