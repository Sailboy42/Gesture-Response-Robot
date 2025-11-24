import { defineConfig } from "vite";import { defineConfig } from "vite";

import react from "@vitejs/plugin-react-swc";import react from "@vitejs/plugin-react-swc";

import path from "path";import path from "path";

import { componentTagger } from "lovable-tagger";import { componentTagger } from "lovable-tagger";



// https://vitejs.dev/config/// https://vitejs.dev/config/

export default defineConfig(({ mode }) => ({export default defineConfig(({ mode }) => ({

  // When deploying to GitHub Pages under a repository path, Vite must  server: {

  // emit asset URLs that include the repo subpath. Set `base` so built    host: "::",

  // files reference "/Gesture-Response-Robot/assets/..." instead of    port: 8080,

  // "/assets/..." which 404s when served from a repo page.  },

  base: "/Gesture-Response-Robot/",  plugins: [react(), mode === "development" && componentTagger()].filter(Boolean),

  server: {  resolve: {

    host: "::",    alias: {

    port: 8080,      "@": path.resolve(__dirname, "./src"),

  },    },

  plugins: [react(), mode === "development" && componentTagger()].filter(Boolean),  },

  resolve: {}));

    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
  },
}));
