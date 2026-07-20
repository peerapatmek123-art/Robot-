import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({

  plugins: [react()],

  base: "./",

  assetsInclude: ["**/*.glb"],

  build: {
    outDir: "dist",
  },

});
