# Robotic Arm Control (Desktop App)

โปรแกรมควบคุมแขนกล 6 แกน — รันเป็นโปรแกรมบนคอมพิวเตอร์จริง (Electron) ไม่ใช่เว็บไซต์
สร้างจากโครงการ "แขนกลต้นทุนต่ำ / Low-Cost RobotArm"

## สิ่งที่ต้องมีก่อน (ติดตั้งครั้งเดียว)

1. ติดตั้ง **Node.js** (แนะนำเวอร์ชัน 18 ขึ้นไป) จาก https://nodejs.org
2. เปิด Terminal / Command Prompt ที่โฟลเดอร์นี้

## ขั้นตอนการรัน

### 1) ติดตั้ง dependencies (ครั้งแรกครั้งเดียว)
```bash
npm install
```
> ขั้นตอนนี้จะดาวน์โหลด Electron ด้วย (~150-200MB) ใช้เวลาสักครู่ตามความเร็วอินเทอร์เน็ต

### 2) รันดูระหว่างพัฒนา (เปิดเป็นโปรแกรมทันที ไม่ต้อง build)
```bash
npm run electron:dev
```
โปรแกรมจะเปิดเป็นหน้าต่างแอปพลิเคชันจริง แก้โค้ดแล้วเห็นผลทันที (hot reload)

### 3) สร้างไฟล์ติดตั้ง (.exe / .dmg / .AppImage) ให้ดาวน์โหลดใช้งานได้จริง
```bash
# Windows (.exe) — ต้องรันคำสั่งนี้บนเครื่อง Windows
npm run dist:win

# macOS (.dmg) — ต้องรันคำสั่งนี้บนเครื่อง Mac
npm run dist:mac

# Linux (.AppImage)
npm run dist:linux
```
ไฟล์ที่ได้จะอยู่ในโฟลเดอร์ `release/` เช่น `Robotic Arm Control Setup 1.0.0.exe`
เอาไฟล์นี้ไปแจกจ่าย/ดาวน์โหลดให้เครื่องอื่นติดตั้งได้เลย ไม่ต้องลง Node.js บนเครื่องปลายทาง

> **สำคัญ:** ต้อง build บนระบบปฏิบัติการเป้าหมาย (build .exe บน Windows, build .dmg บน Mac)
> ถ้าไม่มีเครื่อง Windows แนะนำใช้ GitHub Actions หรือบริการ CI ในการ build ข้ามแพลตฟอร์มได้

## โครงสร้างโปรเจกต์

```
robot-arm-app/
├── electron/
│   ├── main.js       # Electron main process (สร้างหน้าต่างโปรแกรม)
│   └── preload.js     # เปิด API ให้หน้าเว็บเรียกใช้ serial port อย่างปลอดภัย
├── src/
│   ├── RoboticArmControl.jsx   # หน้าตาโปรแกรมทั้งหมด (UI)
│   ├── main.jsx
│   └── index.css
├── index.html
├── package.json
└── vite.config.js
```

## การต่อกับบอร์ด ESP32 จริง (COM Port)

ตอนนี้รายการพอร์ตและการเชื่อมต่อใน `electron/main.js` เป็น **โครงจำลอง (mock)** ไว้ก่อน
เมื่อพร้อมต่อฮาร์ดแวร์จริง ให้ทำตามนี้:

```bash
npm install serialport
```
แล้วเปิดไฟล์ `electron/main.js` และ uncomment โค้ดส่วน `SerialPort` ที่เตรียมไว้ให้แล้ว
(มีคอมเมนต์กำกับไว้ทุกจุดว่าตรงไหนต้องแก้)

## หมายเหตุ

- UI ทั้งหมดอยู่ในไฟล์เดียว `src/RoboticArmControl.jsx` แก้สี/เลย์เอาต์/ข้อความได้ตรงนั้น
- โมเดล 3D ใช้ three.js คำนวณตำแหน่งปลายแขนจริงจากมุมข้อต่อ (ไม่ใช่ค่านิ่ง)
- ไอคอนหน้าต่าง (ย่อ/ขยาย/ปิด) ตอนนี้เป็นแค่ภาพ ยังไม่ผูกกับ Electron window controls จริง
  ถ้าต้องการให้ทำงานจริง แจ้งได้ครับ
