const Hero = () => {
  return (
    <section id="hero" className="min-h-screen flex items-center justify-center pt-20 px-6">
      <div className="container mx-auto text-center animate-fade-in">
        <div className="inline-block mb-4 px-4 py-2 bg-primary/10 rounded-full border border-primary/20">
          <span className="text-primary font-semibold text-sm">Computational Robotics Final Project</span>
        </div>
        <h1 className="text-5xl md:text-7xl font-bold mb-6 bg-gradient-hero bg-clip-text text-transparent">
          Gesture Neato
        </h1>
        <p className="text-xl md:text-2xl text-muted-foreground max-w-3xl mx-auto mb-8">
          A Neato robot that reads hand gestures to perform functions—from following you to taking photos, all controlled with your hands
        </p>
        <div className="flex gap-3 justify-center flex-wrap text-sm mb-6">
          <span className="px-3 py-1 bg-secondary rounded-full text-secondary-foreground font-medium">Tabitha</span>
          <span className="px-3 py-1 bg-secondary rounded-full text-secondary-foreground font-medium">Bhar</span>
          <span className="px-3 py-1 bg-secondary rounded-full text-secondary-foreground font-medium">Khoi</span>
          <span className="px-3 py-1 bg-secondary rounded-full text-secondary-foreground font-medium">Owen</span>
        </div>
        <div className="flex gap-4 justify-center">
          <a 
            href="#showcase" 
            className="px-8 py-3 bg-gradient-hero text-primary-foreground rounded-lg font-semibold hover:shadow-glow transition-all"
          >
            See It In Action
          </a>
          <a 
            href="#architecture" 
            className="px-8 py-3 bg-secondary text-secondary-foreground rounded-lg font-semibold hover:bg-secondary/80 transition-colors"
          >
            View Architecture
          </a>
        </div>
      </div>
    </section>
  );
};

export default Hero;
