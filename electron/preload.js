const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("electronAPI", {
  listPorts: () => ipcRenderer.invoke("serial:list"),
  connectPort: (portName) => ipcRenderer.invoke("serial:connect", portName),
  disconnectPort: () => ipcRenderer.invoke("serial:disconnect"),
  getStatus: () => ipcRenderer.invoke("serial:status"),
  sendJointAngles: (data) => ipcRenderer.invoke("serial:send", data),
});
