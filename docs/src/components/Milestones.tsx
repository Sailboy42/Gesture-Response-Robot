import { Card } from "./ui/card";

const Milestones = () => {
  const milestones = [
    {
      number: 1,
      title: "Camera Integration & Real-Time Hand Tracking",
      date: "Week 1 - Completed",
      description: "Using OpenCV, we created a real-time hand tracking system. The HandTracker class processes webcam video to detect and track a hand using computer vision, establishing a strong foundation for future gesture recognition.",
      achievements: [
        "HandTracker class implemented with OpenCV",
        "Skin detection using HSV color space filtering to identify skin regions",
        "Contour detection finds largest contour and filters out small noise regions",
        "Real-time visualization with detected hand contour overlaid on video feed",
        "Debug view displays skin detection mask for system monitoring",
        "Live video frame processing with noise reduction filters",
        "Green contour successfully drawn around detected hand"
      ],
      challenges: [
        "Camera mounting and positioning for optimal hand detection",
        "Tuning HSV color space thresholds for accurate skin detection",
        "Filtering out noise while maintaining hand contour accuracy",
        "Synchronizing camera feed with ROS environment",
        "Optimizing performance for real-time processing"
      ],
      goals: "✓ Real-time hand tracking system operational / ✓ Skin detection working / ✓ Hand contour visualization complete"
    },
    {
      number: 2,
      title: "Machine Vision Training & Action Programming",
      date: "Week 2 - Planned",
      description: "Next, we will analyze the hand contour to detect individual fingers and the hand center, enabling gesture recognition. We'll train machine vision algorithms to recognize specific gestures and map them to programmed Neato actions.",
      achievements: [
        "Finger detection from hand contour analysis",
        "Hand center point calculation",
        "Gesture recognition algorithm for stop, circle, and snapshot motions",
        "Gesture-to-action mapping system implementation",
        "ROS action server configured for Neato commands",
        "Initial Neato actions programmed and tested"
      ],
      challenges: [
        "Accurately detecting individual fingers from contour data",
        "Differentiating between similar hand poses",
        "Handling variance in how people perform gestures",
        "Balancing recognition speed vs. accuracy",
        "Integrating gesture recognition with robot control"
      ],
      goals: "⏳ Finger and center detection / ⏳ Gesture recognition working / ⏳ Neato responds to gestures"
    },
    {
      number: 3,
      title: "Complete Integration, Obstacle Avoidance & Final Refinement",
      date: "Week 3 - Planned",
      description: "The final milestone will achieve full system integration with all gesture-controlled actions working reliably. We'll implement obstacle avoidance during all programmed actions, ensuring the Neato operates safely while executing gesture-triggered commands with acceptable recognition accuracy.",
      achievements: [
        "All Neato actions coded and operational (stop, spin, photo, follow)",
        "Acceptable gesture recognition success rate",
        "Obstacle avoidance integrated with all gesture actions",
        "LIDAR data processing for real-time obstacle detection",
        "Safe navigation during follow mode",
        "Collision prevention during spin and movement",
        "System integration and testing complete",
        "Documentation and demo materials ready",
        "Final demonstration prepared"
      ],
      challenges: [
        "Fine-tuning recognition thresholds for reliability",
        "Balancing gesture response time with obstacle checking",
        "Handling conflicting priorities (gesture command vs. safety)",
        "Managing system performance with multiple components",
        "Optimizing performance for real-time operation",
        "Testing edge cases and failure modes",
        "Polishing user experience for final demonstration"
      ],
      goals: "⏳ All Neato actions functional / ⏳ Obstacle avoidance integrated / ⏳ Complete system testing / ⏳ Project ready for demo"
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
