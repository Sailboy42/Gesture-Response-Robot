import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { User, Target, Lightbulb, AlertTriangle } from "lucide-react";

interface TeamMember {
  name: string;
  image?: string;
  role: string;
  workedOn: string[];
  learningGoals: string[];
  thingsLearned: string[];
  challenges: string[];
}

const teamMembers: TeamMember[] = [
  {
    name: "Tabitha",
    image: "",
    role: "Camera Integration & ROS Development",
    workedOn: [
      "Set up and debugged camera cable, tripod, and Neato hardware configuration",
      "Wrote the gest_camera_reg node integrating camera feed with gesture analyser",
      "Implemented take_photo function to capture and save images based on detected gestures",
      "Helped debug and integrate functions for reliable robot performance"
    ],
    learningGoals: [
      "Develop stronger understanding of FSM",
      "Become more comfortable with computer vision"
    ],
    thingsLearned: [
      "Significantly improved FSM understanding compared to first project",
      "How to use PiCamera and troubleshoot camera hardware issues",
      "Interfacing camera through ROS and OpenCV",
      "Creating ROS packages and troubleshooting Neato robots",
      "Confidence working across full robotics stack from hardware to software"
    ],
    challenges: [
      "Debugging camera cable and hardware configuration issues",
      "Integrating camera feed reliably with ROS gesture recognition pipeline"
    ]
  },
  {
    name: "Bhar",
    image: "",
    role: "ROS2 Architecture & Integration",
    workedOn: [
      "Implemented existing file structure into ROS2 workspace with new package setup",
      "Created nodes, publishers, and subscribers for Pi camera, Neato, and MediaPipe communication",
      "Built camera detection node connecting MediaPipe ML model to Raspberry Pi camera",
      "Created gestures topic publisher to broadcast recognized gesture names",
      "Developed FSM node with gestures subscriber to trigger Neato actions via cmd_vel topic"
    ],
    learningGoals: [
      "Learn ROS2 workspace and package configuration",
      "Understand publisher/subscriber communication patterns",
      "Develop debugging skills for complex distributed systems"
    ],
    thingsLearned: [
      "Parsing through error messages in setup.py and FSM code",
      "Tracing errors to understand individual components and their dependencies",
      "Finding root causes vs temporary solutions in distributed systems",
      "ROS2 package configuration and node communication patterns"
    ],
    challenges: [
      "Interactions between publishers and subscribers were complex to debug",
      "Configuration issues with ROS2 package setup",
      "Understanding how errors propagate through interconnected components"
    ]
  },
  {
    name: "Khoi",
    image: "",
    role: "Computer Vision & Human Tracking",
    workedOn: [
      "Formatted and integrated OpenCV MediaPipe model for human tracking",
      "Developed hand masking software as foundation",
      "Reformatted MediaPipe pose detection to identify human figures using visual landmarks (hips, shoulders)",
      "Implemented center-of-figure calculation and tracking algorithm",
      "Integrated human following into FSM to orient Neato towards detected figures"
    ],
    learningGoals: [
      "Gain experience with OpenCV frameworks",
      "Learn ROS2 for FSM development",
      "Integrate software with real-world robotics"
    ],
    thingsLearned: [
      "Advanced OpenCV techniques for pose estimation",
      "ROS2 integration for finite state machine code",
      "The satisfaction of connecting software to physical robot behavior"
    ],
    challenges: [
      "The 'I love you' ASL gesture proved too finnicky for final presentation",
      "Balancing gesture recognition accuracy with real-time performance",
      "Feature is near completion but required additional tuning for reliability"
    ]
  },
  {
    name: "Owen",
    image: "",
    role: "Custom ML & Documentation",
    workedOn: [
      "Maintained website and ensured rubric compliance for submissions",
      "Implemented MediaPipe with custom model for hand gesture recognition",
      "Trained custom gesture detection (middle finger) to prove MediaPipe familiarity",
      "Worked in Google Colab using PyTorch ML system for model training",
      "Converted datasets into proper format for MediaPipe intake"
    ],
    learningGoals: [
      "Gain hands-on ML experience with custom model training",
      "Continue developing FSM code skills",
      "Understand dataset management and model training pipelines"
    ],
    thingsLearned: [
      "How to train custom models with MediaPipe rather than relying on provided models",
      "Dataset management and the tradeoffs between accuracy and processing time",
      "Proper Git handling for large datasets to avoid repository issues",
      "Converting datasets to compatible formats for ML pipelines"
    ],
    challenges: [
      "Sourcing and managing proper datasets with thousands of examples",
      "Balancing dataset size with accuracy and processing time",
      "Nearly pushed a 50k file dataset to Git which would have caused major issues",
      "Dataset format conversion for MediaPipe compatibility"
    ]
  }
];

const Team = () => {
  return (
    <section id="team" className="py-20 px-4 bg-muted/30">
      <div className="max-w-6xl mx-auto">
        <div className="text-center mb-16">
          <h2 className="text-4xl md:text-5xl font-bold mb-4">
            Meet the <span className="text-primary">Team</span>
          </h2>
          <p className="text-xl text-muted-foreground max-w-2xl mx-auto">
            The people behind the Gesture Recognition project
          </p>
        </div>

        <div className="grid md:grid-cols-2 gap-8">
          {teamMembers.map((member, index) => (
            <Card key={index} className="overflow-hidden">
              <CardHeader className="text-center pb-4">
                <div className="w-32 h-32 mx-auto mb-4 rounded-full bg-primary/10 flex items-center justify-center overflow-hidden border-4 border-primary/20">
                  {member.image ? (
                    <img 
                      src={member.image} 
                      alt={member.name}
                      className="w-full h-full object-cover"
                    />
                  ) : (
                    <User className="w-16 h-16 text-primary/50" />
                  )}
                </div>
                <CardTitle className="text-2xl">{member.name}</CardTitle>
                <CardDescription className="text-lg">{member.role}</CardDescription>
              </CardHeader>
              
              <CardContent className="space-y-6">
                {/* What I Worked On */}
                <div className="space-y-2">
                  <div className="flex items-center gap-2 text-primary font-semibold">
                    <Target className="w-5 h-5" />
                    <h4>What I Worked On</h4>
                  </div>
                  <ul className="list-disc list-inside text-muted-foreground space-y-1 pl-2">
                    {member.workedOn.map((item, i) => (
                      <li key={i}>{item}</li>
                    ))}
                  </ul>
                </div>

                {/* Learning Goals */}
                <div className="space-y-2">
                  <div className="flex items-center gap-2 text-primary font-semibold">
                    <Target className="w-5 h-5" />
                    <h4>Learning Goals</h4>
                  </div>
                  <ul className="list-disc list-inside text-muted-foreground space-y-1 pl-2">
                    {member.learningGoals.map((goal, i) => (
                      <li key={i}>{goal}</li>
                    ))}
                  </ul>
                </div>

                {/* Things I Learned */}
                <div className="space-y-2">
                  <div className="flex items-center gap-2 text-green-600 font-semibold">
                    <Lightbulb className="w-5 h-5" />
                    <h4>Things I Learned</h4>
                  </div>
                  <ul className="list-disc list-inside text-muted-foreground space-y-1 pl-2">
                    {member.thingsLearned.map((learning, i) => (
                      <li key={i}>{learning}</li>
                    ))}
                  </ul>
                </div>

                {/* Challenges */}
                <div className="space-y-2">
                  <div className="flex items-center gap-2 text-orange-500 font-semibold">
                    <AlertTriangle className="w-5 h-5" />
                    <h4>Challenges</h4>
                  </div>
                  <ul className="list-disc list-inside text-muted-foreground space-y-1 pl-2">
                    {member.challenges.map((challenge, i) => (
                      <li key={i}>{challenge}</li>
                    ))}
                  </ul>
                </div>
              </CardContent>
            </Card>
          ))}
        </div>
      </div>
    </section>
  );
};

export default Team;
