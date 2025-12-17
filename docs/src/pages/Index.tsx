import Navigation from "@/components/Navigation";
import Hero from "@/components/Hero";
import Showcase from "@/components/Showcase";
import Architecture from "@/components/Architecture";
import Milestones from "@/components/Milestones";
import Team from "@/components/Team";
import Footer from "@/components/Footer";

const Index = () => {
  return (
    <div className="min-h-screen bg-background">
      <Navigation />
      <Hero />
      <Showcase />
      <Architecture />
      <Milestones />
      <Team />
      <Footer />
    </div>
  );
};

export default Index;
