import { defineConfig } from "vite";
import react from "@vitejs/plugin-react-swc";
import path from "path";
import { componentTagger } from "lovable-tagger";


// https://vitejs.dev/config/
export default defineConfig(({ mode }) => ({

  // When deploying to GitHub Pages at
  // https://Sailboy42.github.io/Gesture-Response-Robot/ we must set the
  // base to the repository name so asset URLs resolve correctly. This
  // ensures built files reference "/Gesture-Response-Robot/assets/..."
  // instead of "/assets/..." which would 404 for repo sites.
  base: "/Gesture-Response-Robot/",

  server: {
    host: "::",
    port: 8080,
  },

  plugins: [react(), mode === "development" && componentTagger()].filter(
    Boolean
  ),

  resolve: {
    alias: {
      "@": path.resolve(__dirname, "./src"),
    },
  },
}));
