import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  base: "./", // สำคัญ: ให้ path ไฟล์ทำงานถูกต้องเมื่อรันใน Electron (file://)
  build: {
    outDir: "dist",
  },
});
