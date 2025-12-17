import { Card } from "./ui/card";

const Milestones = () => {
  const milestones = [
    {
      number: 1,
      title: "Camera Integration & Real-Time Hand Tracking",
      date: "Week 1 - Completed",

  description: "This past week, we focused on getting the camera feed working. We purchased a 1-meter ribbon cable that's compatible with the Neato's camera, and we've now managed to bring the feed up in rqt. Our next step is figuring out how to save that feed as a bag file for further testing. In the meantime, we also have code running that uses a regular webcam so the team can continue development until we have access to the Neato again after break. Using OpenCV, we created a real-time hand tracking system that processes video to detect and track hands, establishing a strong foundation for future gesture recognition.",
      achievements: [
        "Successfully purchased and connected 1-meter ribbon cable compatible with Neato's camera",
        "Camera feed successfully brought up in rqt for visualization",
        "Backup webcam code implemented for continued development during break",
        "HandTracker class implemented with OpenCV",
        "Skin detection using HSV color space filtering to identify skin regions",
        "Contour detection finds largest contour and filters out small noise regions",
        "Real-time visualization with detected hand contour overlaid on video feed",
        "Debug view displays skin detection mask for system monitoring",
        "Researched two gesture recognition methods: Keypoint-Based Tracking (finger tip keypoints with position/angle comparison) and Contour/Convex Hull Analysis (hand segmentation with convexity defect detection)"
      ],
      challenges: [
        "Working on saving camera feed as bag file for further testing",
        "Camera mounting and positioning for optimal hand detection",
        "Tuning HSV color space thresholds for accurate skin detection",
        "Filtering out noise while maintaining hand contour accuracy",
        "Synchronizing camera feed with ROS environment",
        "Comparing gesture recognition approaches: keypoint-based vs. convex hull analysis"
      ],
      goals: "✓ Camera feed operational in rqt / ✓ Webcam backup system ready / ✓ Real-time hand tracking working / ⏳ Save feed as bag file"
    },
    {
      number: 2,
      title: "MediaPipe Integration & FSM Implementation",
      date: "Week 2 - Completed",
      description: "We implemented Google AI's MediaPipe Model to take live camera footage and recognize predetermined gestures. We began by using the computer webcam footage from the previous milestone and converting each frame into images in a format that could be received as input by the MediaPipe Model. We then adjusted the output of the model to give only the relevant output for our project—the gesture identified. After a few iterations, we were able to get the model to successfully detect and output the set of 7 gestures when an individual's hand was identified in frame. The gesture detected output serves as input and triggers for the Neato's finite state machine.",
      achievements: [
        "Integrated PiCamera and long ribbon cable—both now work seamlessly with the gesture-analysis pipeline",
        "Combined gesture analyzer and PiCamera code so gestures are correctly detected and passed into the FSM to trigger state changes",
        "Successfully added functionality to capture and save an image to Downloads when fist gesture is recognized",
        "Implemented Google AI's MediaPipe Model for gesture recognition",
        "Converted webcam frames to MediaPipe-compatible image format",
        "Model successfully detects and outputs 7 predetermined gestures",
        "Gesture output integrated as triggers for Neato's finite state machine"
      ],
      challenges: [
        "Converting video frames to MediaPipe-compatible format",
        "Filtering model output to only relevant gesture data",
        "Iterating on model parameters for reliable gesture detection",
        "Synchronizing PiCamera feed with gesture analysis pipeline",
        "Mapping gesture outputs to FSM state transitions"
      ],
      goals: "✓ PiCamera integrated with pipeline / ✓ MediaPipe gesture recognition working / ✓ 7 gestures detected / ✓ FSM triggers functional"
    },
    {
      number: 3,
      title: "Full System Integration & Custom ML Development",
      date: "Week 3 - Completed",
      description: "The final milestone brought together all components into a cohesive ROS2 workspace. Bhar structured the entire file system into a ROS2 package with nodes, publishers, and subscribers enabling communication between the Pi camera, Neato, and MediaPipe model. Tabitha finalized the gest_camera_reg node integrating the camera feed with gesture analysis and implemented the take_photo function. Khoi developed a human-following feature using MediaPipe pose detection to track and follow humans via the 'iloveyou' ASL gesture—though deemed too finnicky for MVP, it's near completion. Owen trained a custom MediaPipe model using PyTorch in Google Colab to prove familiarity beyond the provided models.",
      achievements: [
        "ROS2 workspace fully structured with custom package, nodes, publishers, and subscribers",
        "Camera detection node connecting MediaPipe ML model to Raspberry Pi camera operational",
        "Gestures topic publisher broadcasting recognized gesture names to FSM",
        "FSM node with gestures subscriber triggering Neato actions via cmd_vel topic",
        "gest_camera_reg node integrating camera feed with gesture analyzer complete",
        "take_photo function captures and saves images on gesture trigger",
        "Human tracking using MediaPipe pose detection with landmark-based center calculation",
        "Neato orientation logic for following detected human figures implemented",
        "Custom MediaPipe model trained in Google Colab with PyTorch",
        "Dataset conversion pipeline for MediaPipe intake completed",
        "Website maintained and rubric compliance ensured"
      ],
      challenges: [
        "Publisher/subscriber interactions required extensive debugging",
        "ROS2 package configuration issues with setup.py and node communication",
        "Human-following feature too finnicky for final presentation, kept for future work",
        "Sourcing proper datasets with thousands of examples for custom model training",
        "Nearly pushed 50k file dataset to Git—learned importance of Git handling for large files",
        "Balancing dataset size with accuracy and processing time tradeoffs",
        "Tracing errors through interconnected ROS2 components to find root causes"
      ],
      goals: "✓ ROS2 workspace structured / ✓ All nodes communicating / ✓ Custom ML model trained / ✓ Human tracking developed / ✓ System demo ready"
    }
  ];

  return (
    <section id="milestones" className="py-20 px-6 bg-muted/30">
      <div className="container mx-auto">
        <h2 className="text-4xl md:text-5xl font-bold mb-4 text-center">Project Milestones</h2>
        <p className="text-xl text-muted-foreground text-center mb-12 max-w-3xl mx-auto">
          The journey from concept to working system
        </p>

        <div className="max-w-5xl mx-auto space-y-8">
          {milestones.map((milestone, index) => (
            <Card key={milestone.number} className="p-8 shadow-elevation hover:shadow-glow transition-all animate-fade-in" style={{ animationDelay: `${index * 0.1}s` }}>
              <div className="flex items-start gap-6">
                <div className="flex-shrink-0">
                  <div className="w-16 h-16 bg-gradient-hero rounded-full flex items-center justify-center text-2xl font-bold text-primary-foreground">
                    {milestone.number}
                  </div>
                </div>
                
                <div className="flex-grow">
                  <div className="flex items-center gap-4 mb-3 flex-wrap">
                    <h3 className="text-2xl font-bold">{milestone.title}</h3>
                    <span className="px-3 py-1 bg-accent/10 text-accent text-sm font-semibold rounded-full">
                      {milestone.date}
                    </span>
                  </div>
                  
                  <p className="text-muted-foreground mb-4 leading-relaxed">
                    {milestone.description}
                  </p>

                  <div className="mb-4 p-3 bg-primary/5 rounded-lg border border-primary/20">
                    <p className="text-sm font-semibold text-primary">{milestone.goals}</p>
                  </div>

                  <div className="grid md:grid-cols-2 gap-6">
                    <div>
                      <h4 className="font-semibold text-primary mb-3 flex items-center gap-2">
                        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 12l2 2 4-4m6 2a9 9 0 11-18 0 9 9 0 0118 0z" />
                        </svg>
                        Key Achievements
                      </h4>
                      <ul className="space-y-2">
                        {milestone.achievements.map((achievement, i) => (
                          <li key={i} className="text-sm text-muted-foreground flex items-start gap-2">
                            <span className="text-primary mt-1">•</span>
                            <span>{achievement}</span>
                          </li>
                        ))}
                      </ul>
                    </div>

                    <div>
                      <h4 className="font-semibold text-accent mb-3 flex items-center gap-2">
                        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 9v2m0 4h.01m-6.938 4h13.856c1.54 0 2.502-1.667 1.732-3L13.732 4c-.77-1.333-2.694-1.333-3.464 0L3.34 16c-.77 1.333.192 3 1.732 3z" />
                        </svg>
                        Challenges Faced
                      </h4>
                      <ul className="space-y-2">
                        {milestone.challenges.map((challenge, i) => (
                          <li key={i} className="text-sm text-muted-foreground flex items-start gap-2">
                            <span className="text-accent mt-1">•</span>
                            <span>{challenge}</span>
                          </li>
                        ))}
                      </ul>
                    </div>
                  </div>
                </div>
              </div>
            </Card>
          ))}
        </div>

        <Card className="mt-12 p-8 shadow-elevation max-w-5xl mx-auto bg-gradient-to-br from-accent/5 to-primary/5 border-accent/20">
          <h3 className="text-2xl font-bold mb-4">Project Risks & Mitigation</h3>
          <p className="text-muted-foreground mb-4">Throughout development, we identified and addressed several key risks:</p>
          <ul className="space-y-3 text-muted-foreground">
            <li className="flex items-start gap-3">
              <span className="text-accent font-bold text-lg">1.</span>
              <span><strong>Camera feed integration:</strong> Overcame by testing multiple connection methods and selecting the most stable configuration for ROS streaming.</span>
            </li>
            <li className="flex items-start gap-3">
              <span className="text-accent font-bold text-lg">2.</span>
              <span><strong>Gesture ambiguity:</strong> Managed by carefully designing distinct gestures and implementing confidence thresholding.</span>
            </li>
            <li className="flex items-start gap-3">
              <span className="text-accent font-bold text-lg">3.</span>
              <span><strong>System integration complexity:</strong> Addressed through modular development and incremental testing of each component.</span>
            </li>
            <li className="flex items-start gap-3">
              <span className="text-accent font-bold text-lg">4.</span>
              <span><strong>Recognition variance:</strong> Improved by collecting diverse training data and tuning classification parameters.</span>
            </li>
            <li className="flex items-start gap-3">
              <span className="text-accent font-bold text-lg">5.</span>
              <span><strong>Hardware limitations:</strong> Mitigated by optimizing camera placement and ensuring robust network connectivity.</span>
            </li>
          </ul>
        </Card>
      </div>
    </section>
  );
};

export default Milestones;
