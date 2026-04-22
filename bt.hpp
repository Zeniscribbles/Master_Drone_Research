#ifndef BT_HPP
#define BT_HPP


#include "behaviortree_cpp/bt_factory.h"


static const char* xml_text = R"(
<root BTCPP_format="4" >

    <BehaviorTree ID="MainTree">
        <Sequence name="root">
            <AlwaysSuccess/>
            <SaySomething   message="this works too" />
            <ThinkWhatToSay text="{the_answer}"/>
            <SaySomething   message="{the_answer}" />
        </Sequence>
    </BehaviorTree>

</root>
)";


class SaySomething : public BT::StatefulActionNode
{
public:
    SaySomething(const std::string &name, const BT::NodeConfig &config) : BT::StatefulActionNode(name, config)
    {
    }

    BT::NodeStatus onStart() override
    {
        std::string msg;
        getInput("message", msg);
        std::cout << msg << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    BT::NodeStatus onRunning() override {return BT::NodeStatus::SUCCESS;}
    void onHalted() override {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("message")};
    }
};

class ThinkWhatToSay : public BT::StatefulActionNode
{
public:
    ThinkWhatToSay(const std::string &name, const BT::NodeConfig &config) : BT::StatefulActionNode(name, config)
    {
    }

    BT::NodeStatus onStart() override
    {
        setOutput("text", "The answer is 42");
        return BT::NodeStatus::SUCCESS;
    }
    BT::NodeStatus onRunning() override {return BT::NodeStatus::SUCCESS;}
    void onHalted() override {}

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<std::string>("text")};
    }
};


#endif