import { Card } from "./ui/card";

const Showcase = () => {
  return (
    <section id="showcase" className="py-20 px-6 bg-muted/30">
      <div className="container mx-auto">
        <h2 className="text-4xl md:text-5xl font-bold mb-4 text-center">Show It Off</h2>
        <p className="text-xl text-muted-foreground text-center mb-12 max-w-3xl mx-auto">
          Experience our gesture recognition system in action
        </p>
        
        <div className="grid md:grid-cols-2 gap-8 mb-12">
          <Card className="p-8 shadow-elevation hover:shadow-glow transition-all">
            <h3 className="text-2xl font-bold mb-4 text-primary">What Does It Do?</h3>
            <p className="text-muted-foreground leading-relaxed">
              Gesture Neato uses an external camera and computer vision to recognize hand gestures and control a Neato robot in real-time. 
              Our system recognizes specific gestures including: a <strong>stop hand</strong> to halt the robot, <strong>circling your finger</strong> to make it spin, 
              and a <strong>snapshot motion</strong> to trigger the camera. The robot processes these visual commands and responds with corresponding movements 
              and actions, all while maintaining obstacle avoidance.
            </p>
          </Card>

          <Card className="p-8 shadow-elevation hover:shadow-glow transition-all">
            <h3 className="text-2xl font-bold mb-4 text-primary">Why This Matters</h3>
            <p className="text-muted-foreground leading-relaxed">
              This project combines practical robotics with intuitive human-computer interaction. By eliminating the need for traditional controllers 
              or voice commands, gesture control creates a more natural and engaging way to interact with robots. This has applications in assistive 
              technology, hands-free operation in hazardous environments, and making robotics more accessible to users unfamiliar with complex control systems. 
              Our scalable design allows for expanding the gesture vocabulary and robot capabilities.
            </p>
          </Card>
        </div>

        <Card className="p-8 shadow-elevation">
          <h3 className="text-2xl font-bold mb-6 text-primary">System Demo</h3>
          <div className="aspect-video bg-secondary rounded-lg flex items-center justify-center mb-6">
            <div className="text-center text-muted-foreground">
              <svg className="w-20 h-20 mx-auto mb-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z" />
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />
              </svg>
              <p className="text-lg">Video demonstration will be embedded here</p>
              <p className="text-sm mt-2">Replace this with your project demo video</p>
            </div>
          </div>
          
          <h4 className="text-xl font-semibold mb-4">Major Components</h4>
          <div className="grid md:grid-cols-4 gap-4">
            <div className="p-4 bg-background rounded-lg border border-border">
              <div className="w-12 h-12 bg-gradient-hero rounded-lg mb-3 flex items-center justify-center">
                <svg className="w-6 h-6 text-primary-foreground" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 10l4.553-2.276A1 1 0 0121 8.618v6.764a1 1 0 01-1.447.894L15 14M5 18h8a2 2 0 002-2V8a2 2 0 00-2-2H5a2 2 0 00-2 2v8a2 2 0 002 2z" />
                </svg>
              </div>
              <h5 className="font-semibold mb-2">External Camera</h5>
              <p className="text-sm text-muted-foreground">Captures video feed and streams to ROS environment</p>
            </div>
            
            <div className="p-4 bg-background rounded-lg border border-border">
              <div className="w-12 h-12 bg-gradient-accent rounded-lg mb-3 flex items-center justify-center">
                <svg className="w-6 h-6 text-accent-foreground" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
                </svg>
              </div>
              <h5 className="font-semibold mb-2">Computer Vision</h5>
              <p className="text-sm text-muted-foreground">Machine vision algorithms to detect and classify gestures</p>
            </div>
            
            <div className="p-4 bg-background rounded-lg border border-border">
              <div className="w-12 h-12 bg-gradient-hero rounded-lg mb-3 flex items-center justify-center">
                <svg className="w-6 h-6 text-primary-foreground" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
                </svg>
              </div>
              <h5 className="font-semibold mb-2">Path Planning</h5>
              <p className="text-sm text-muted-foreground">Commands-based navigation with obstacle avoidance</p>
            </div>
            
            <div className="p-4 bg-background rounded-lg border border-border">
              <div className="w-12 h-12 bg-gradient-accent rounded-lg mb-3 flex items-center justify-center">
                <svg className="w-6 h-6 text-accent-foreground" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
              </div>
              <h5 className="font-semibold mb-2">Neato Control</h5>
              <p className="text-sm text-muted-foreground">Robot actions: follow, stop, spin, and photo capture</p>
            </div>
          </div>
        </Card>
      </div>
    </section>
  );
};

export default Showcase;
