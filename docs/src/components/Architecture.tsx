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
                1. Camera & ROS Integration
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>External camera mounted and connected to laptop</li>
                    <li>Camera feed published to ROS environment via image topics</li>
                    <li>Real-time video streaming with frame synchronization</li>
                    <li>Image preprocessing for gesture recognition pipeline</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Algorithms:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>OpenCV video capture and frame processing</li>
                    <li>ROS image transport for efficient data streaming</li>
                    <li>Color space conversion and noise filtering</li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>

            <AccordionItem value="detection" className="border border-border rounded-lg px-6 bg-card shadow-elevation">
              <AccordionTrigger className="text-xl font-semibold hover:text-primary">
                2. Machine Vision & Gesture Recognition
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Machine vision training pipeline for gesture classification</li>
                    <li>Gesture recognition node subscribed to camera feed</li>
                    <li>Real-time gesture detection and validation</li>
                    <li>Command publishing to Neato control node</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Algorithms:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Hand detection and segmentation using computer vision</li>
                    <li>Gesture differentiation through feature comparison</li>
                    <li>Motion detection for dynamic gestures (circling, snapshot)</li>
                    <li>Confidence thresholding to reduce false positives</li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>

            <AccordionItem value="features" className="border border-border rounded-lg px-6 bg-card shadow-elevation">
              <AccordionTrigger className="text-xl font-semibold hover:text-primary">
                3. Path Planning & Navigation
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Command-based path planning system</li>
                    <li>Person tracking and following behavior</li>
                    <li>Obstacle avoidance integration during movement</li>
                    <li>Velocity and angular control for Neato actions</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Algorithms:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Target tracking using visual servoing</li>
                    <li>Potential field method for obstacle avoidance</li>
                    <li>PID control for smooth movement execution</li>
                    <li>Dynamic path replanning based on sensor feedback</li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>

            <AccordionItem value="classification" className="border border-border rounded-lg px-6 bg-card shadow-elevation">
              <AccordionTrigger className="text-xl font-semibold hover:text-primary">
                4. Neato Action Controller
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>ROS node for Neato motor control</li>
                    <li>Gesture-to-action mapping system</li>
                    <li>State machine for managing robot behaviors</li>
                    <li>Safety checks and emergency stop functionality</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Implemented Actions:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li><strong>Stop gesture:</strong> Immediate halt of all movement</li>
                    <li><strong>Circle gesture:</strong> Spin in place at controlled angular velocity</li>
                    <li><strong>Snapshot gesture:</strong> Trigger camera to capture image</li>
                    <li><strong>Follow mode:</strong> Track and follow detected person</li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>

            <AccordionItem value="output" className="border border-border rounded-lg px-6 bg-card shadow-elevation">
              <AccordionTrigger className="text-xl font-semibold hover:text-primary">
                5. Obstacle Avoidance & Safety
              </AccordionTrigger>
              <AccordionContent className="text-muted-foreground pt-4 space-y-4">
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Code Structure:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>LIDAR sensor data processing</li>
                    <li>Real-time obstacle detection during movement</li>
                    <li>Integration with path planning for dynamic avoidance</li>
                    <li>Safety override for gesture-controlled actions</li>
                  </ul>
                </div>
                <div>
                  <h4 className="font-semibold text-foreground mb-2">Algorithms:</h4>
                  <ul className="list-disc list-inside space-y-1 ml-4">
                    <li>Proximity detection using LIDAR scan data</li>
                    <li>Vector field histogram for local navigation</li>
                    <li>Dynamic window approach for velocity planning</li>
                    <li>Emergency stop triggers based on collision risk</li>
                  </ul>
                </div>
              </AccordionContent>
            </AccordionItem>
          </Accordion>

          <Card className="mt-12 p-8 shadow-elevation bg-gradient-to-br from-primary/5 to-accent/5 border-primary/20">
            <h3 className="text-2xl font-bold mb-4">System Integration</h3>
            <p className="text-muted-foreground leading-relaxed mb-4">
              All components are integrated through ROS (Robot Operating System), allowing modular communication between 
              nodes. The camera feed flows through the gesture recognition pipeline, which publishes command messages to the 
              Neato control node. The robot simultaneously processes LIDAR data for obstacle avoidance, ensuring safe execution 
              of gesture-triggered actions.
            </p>
            <p className="text-muted-foreground leading-relaxed">
              The scalable architecture allows us to add new gestures incrementally. Each gesture can be developed and tested 
              independently before integration into the full system, making the project highly iterable and expandable.
            </p>
          </Card>
        </div>
      </div>
    </section>
  );
};

export default Architecture;
