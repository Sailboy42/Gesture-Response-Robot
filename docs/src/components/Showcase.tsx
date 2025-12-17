import { Card } from "./ui/card";

const Showcase = () => {
  const baseUrl = import.meta.env.BASE_URL;
  const videoSrc = `${baseUrl}videos/demo.mp4`;
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
              Gesture Neato uses a PiCamera with a long ribbon cable and Google's MediaPipe ML model to recognize hand gestures and control a Neato robot in real-time via ROS2. 
              Our system recognizes <strong>7 predetermined gestures</strong> including: an <strong>open palm</strong> to halt the robot, a <strong>fist</strong> to capture a photo, 
              a <strong>pointing gesture</strong> to make it spin, and an <strong>"I love you" ASL sign</strong> to trigger human-following mode. The gestures are published to a dedicated 
              ROS2 topic and processed by a finite state machine that triggers corresponding Neato actions via the cmd_vel topic.
            </p>
          </Card>

          <Card className="p-8 shadow-elevation hover:shadow-glow transition-all">
            <h3 className="text-2xl font-bold mb-4 text-primary">Why This Matters</h3>
            <p className="text-muted-foreground leading-relaxed">
              This project combines practical robotics with intuitive human-computer interaction using modern ML frameworks. By leveraging MediaPipe's gesture recognition 
              and building a complete ROS2 workspace with custom nodes, publishers, and subscribers, we created a modular system that's both powerful and extensible. 
              We also trained a custom PyTorch model in Google Colab to prove familiarity beyond MediaPipe's provided models. This has applications in assistive 
              technology, hands-free operation, and making robotics more accessible to users unfamiliar with complex control systems.
            </p>
          </Card>
        </div>

        <Card className="p-8 shadow-elevation">
          <h3 className="text-2xl font-bold mb-6 text-primary">System Demo</h3>
          <div className="flex justify-center mb-6">
            <div className="aspect-[9/16] max-h-[500px] bg-secondary rounded-lg overflow-hidden">
              <video 
                className="w-full h-full object-contain"
                controls
              >
                <source src={videoSrc} type="video/mp4" />
                Your browser does not support the video tag.
              </video>
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
              <h5 className="font-semibold mb-2">PiCamera + ROS2</h5>
              <p className="text-sm text-muted-foreground">Raspberry Pi camera with long ribbon cable streaming to ROS2 via gest_camera_reg node</p>
            </div>
            
            <div className="p-4 bg-background rounded-lg border border-border">
              <div className="w-12 h-12 bg-gradient-accent rounded-lg mb-3 flex items-center justify-center">
                <svg className="w-6 h-6 text-accent-foreground" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
                </svg>
              </div>
              <h5 className="font-semibold mb-2">MediaPipe ML</h5>
              <p className="text-sm text-muted-foreground">Google's MediaPipe model detecting 7 gestures, publishing to gestures topic</p>
            </div>
            
            <div className="p-4 bg-background rounded-lg border border-border">
              <div className="w-12 h-12 bg-gradient-hero rounded-lg mb-3 flex items-center justify-center">
                <svg className="w-6 h-6 text-primary-foreground" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" />
                </svg>
              </div>
              <h5 className="font-semibold mb-2">Finite State Machine</h5>
              <p className="text-sm text-muted-foreground">FSM node subscribes to gestures topic and triggers Neato actions</p>
            </div>
            
            <div className="p-4 bg-background rounded-lg border border-border">
              <div className="w-12 h-12 bg-gradient-accent rounded-lg mb-3 flex items-center justify-center">
                <svg className="w-6 h-6 text-accent-foreground" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
              </div>
              <h5 className="font-semibold mb-2">Neato Control</h5>
              <p className="text-sm text-muted-foreground">Robot actions via cmd_vel: stop, spin, photo capture, and human following</p>
            </div>
          </div>
        </Card>
      </div>
    </section>
  );
};

export default Showcase;
