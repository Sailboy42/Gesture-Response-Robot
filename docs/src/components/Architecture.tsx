import { Card } from "./ui/card";
import { Accordion, AccordionContent, AccordionItem, AccordionTrigger } from "./ui/accordion";

const Architecture = () => {
  return (
    <section id="architecture" className="py-20 px-6">
      <div className="container mx-auto">
        <h2 className="text-4xl md:text-5xl font-bold mb-4 text-center">System Architecture</h2>
        <p className="text-xl text-muted-foreground text-center mb-12 max-w-3xl mx-auto">
          Deep dive into the technical implementation and algorithms
        </p>

        <div className="max-w-4xl mx-auto">
          <Accordion type="single" collapsible className="space-y-4">
            <AccordionItem value="input" className="border border-border rounded-lg px-6 bg-card shadow-elevation">
              <AccordionTrigger className="text-xl font-semibold hover:text-primary">
                1. PiCamera & ROS2 Integration
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Raspberry Pi camera with 1-meter ribbon cable connected to Neato</li>
                    <li><code className="bg-muted px-1 rounded">gest_camera_reg</code> node integrates camera feed with gesture analyzer</li>
                    <li>Camera feed brought up in rqt for visualization and debugging</li>
                    <li>Backup webcam code for development when Neato unavailable</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Algorithms:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>OpenCV video capture and frame processing</li>
                    <li>Frame conversion to MediaPipe-compatible image format</li>
                    <li>HSV color space filtering for skin detection (early development)</li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>

            <AccordionItem value="detection" className="border border-border rounded-lg px-6 bg-card shadow-elevation">
              <AccordionTrigger className="text-xl font-semibold hover:text-primary">
                2. MediaPipe Gesture Recognition
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Google's MediaPipe ML model for hand gesture detection</li>
                    <li>Camera detection node connecting MediaPipe to Raspberry Pi camera</li>
                    <li>Publisher to <code className="bg-muted px-1 rounded">gestures</code> topic broadcasting recognized gesture names (String type)</li>
                    <li>Custom PyTorch model trained in Google Colab for extended gesture vocabulary</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Recognized Gestures (7 total):</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Open palm, closed fist, pointing up, thumbs up/down</li>
                    <li>"I love you" ASL sign for human-following mode</li>
                    <li>Custom middle finger gesture (trained via PyTorch to prove MediaPipe familiarity)</li>
                    <li>Confidence thresholding to reduce false positives</li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>

            <AccordionItem value="features" className="border border-border rounded-lg px-6 bg-card shadow-elevation">
              <AccordionTrigger className="text-xl font-semibold hover:text-primary">
                3. Human Tracking & Following
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>MediaPipe pose detection reformatted from hand to full human figure</li>
                    <li>Visual landmarks: right/left hip, right/left shoulder for figure detection</li>
                    <li>Center-of-figure calculation for each frame</li>
                    <li>Neato orientation logic to drive toward detected human</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Algorithms:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Pose landmark detection using MediaPipe pose model</li>
                    <li>Bounding box center calculation from shoulder/hip landmarks</li>
                    <li>Turn-then-drive approach: orient first, then move straight toward figure</li>
                    <li><em>Note: Feature developed but deemed too finnicky for MVP presentation</em></li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>

            <AccordionItem value="classification" className="border border-border rounded-lg px-6 bg-card shadow-elevation">
              <AccordionTrigger className="text-xl font-semibold hover:text-primary">
                4. Finite State Machine & Neato Control
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>FSM node with subscriber to <code className="bg-muted px-1 rounded">gestures</code> topic</li>
                    <li>Gesture-to-action mapping triggers Neato movement</li>
                    <li>Commands published to <code className="bg-muted px-1 rounded">cmd_vel</code> topic (Neato subscribed)</li>
                    <li><code className="bg-muted px-1 rounded">take_photo</code> function captures and saves images to Downloads</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Implemented Actions:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li><strong>Open palm:</strong> Immediate halt of all movement</li>
                    <li><strong>Pointing gesture:</strong> Spin in place at controlled angular velocity</li>
                    <li><strong>Fist gesture:</strong> Trigger camera to capture and save image</li>
                    <li><strong>"I love you" ASL:</strong> Activate human-following mode</li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>

            <AccordionItem value="output" className="border border-border rounded-lg px-6 bg-card shadow-elevation">
              <AccordionTrigger className="text-xl font-semibold hover:text-primary">
                5. ROS2 Workspace Architecture
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Custom ROS2 package with proper setup.py configuration</li>
                    <li>Nodes for camera detection, gesture publishing, and FSM control</li>
                    <li>Publishers/subscribers connecting Pi camera, Neato, Downloads folder, and MediaPipe</li>
                    <li>Modular architecture allowing independent component development</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Communication Flow:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Camera node → MediaPipe → Gesture publisher → <code className="bg-muted px-1 rounded">gestures</code> topic</li>
                    <li>FSM subscriber → Action logic → <code className="bg-muted px-1 rounded">cmd_vel</code> publisher → Neato</li>
                    <li>Photo capture → Downloads folder save</li>
                    <li>All components debuggable via ROS2 tools and rqt visualization</li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>
          </Accordion>

          <Card className="mt-12 p-8 shadow-elevation bg-gradient-to-br from-primary/5 to-accent/5 border-primary/20">
            <h3 className="text-2xl font-bold mb-4">System Integration</h3>
            <p className="text-muted-foreground leading-relaxed mb-4">
              All components are integrated through ROS2, with a custom package containing nodes, publishers, and subscribers 
              for modular communication. The PiCamera feed flows through the <code className="bg-muted px-1 rounded">gest_camera_reg</code> node 
              to MediaPipe, which publishes recognized gestures to the <code className="bg-muted px-1 rounded">gestures</code> topic. 
              The FSM node subscribes to this topic and triggers Neato actions via the <code className="bg-muted px-1 rounded">cmd_vel</code> topic.
            </p>
            <p className="text-muted-foreground leading-relaxed">
              The modular ROS2 architecture allows each component to be developed and tested independently. We also demonstrated 
              extensibility by training a custom PyTorch model in Google Colab, proving we understand MediaPipe beyond its provided models. 
              The human-following feature using pose detection is near completion and can be integrated when reliability improves.
            </p>
          </Card>
        </div>
      </div>
    </section>
  );
};

export default Architecture;
