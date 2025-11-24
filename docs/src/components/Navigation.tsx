import { NavLink } from "./NavLink";
import { Github } from "lucide-react";

const Navigation = () => {
  const navItems = [
    { to: "#hero", label: "Overview" },
    { to: "#showcase", label: "Show It Off" },
    { to: "#architecture", label: "Architecture" },
    { to: "#milestones", label: "Milestones" },
  ];

  return (
    <nav className="fixed top-0 left-0 right-0 z-50 bg-background/80 backdrop-blur-lg border-b border-border">
      <div className="container mx-auto px-6 py-4">
        <div className="flex items-center justify-between">
          <a href="#hero" className="text-2xl font-bold bg-gradient-hero bg-clip-text text-transparent">
            Gesture Neato
          </a>
          <div className="hidden md:flex gap-8 items-center">
            {navItems.map((item) => (
              <a
                key={item.to}
                href={item.to}
                className="text-foreground/80 hover:text-primary transition-colors font-medium"
              >
                {item.label}
              </a>
            ))}
            <a
              href="https://github.com/Sailboy42/Gesture-Response-Robot"
              target="_blank"
              rel="noopener noreferrer"
              className="text-foreground/80 hover:text-primary transition-colors"
              aria-label="View GitHub Repository"
            >
              <Github className="w-5 h-5" />
            </a>
          </div>
        </div>
      </div>
    </nav>
  );
};

export default Navigation;
