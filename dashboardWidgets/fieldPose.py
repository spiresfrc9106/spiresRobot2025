from dashboardWidgets.widgetConfig import WidgetConfig
from utils.signalLogging import sigNameToNT4TopicName


# Private helper class: Describes a set of topics associated with one module

class FieldPose(WidgetConfig):
    def __init__(self, xPos, yPos):
        WidgetConfig.__init__(self, None, xPos, yPos)
        self.nominalHeight = 15
        self.nominalWidth = 32.7
        self.isVisible = True
        self.x_pos = sigNameToNT4TopicName("ytest_position_final_x")
        self.y_pos = sigNameToNT4TopicName("ytest_position_final_y")
        self.t_pos = sigNameToNT4TopicName("ytest_position_final_t")

    def getJSDeclaration(self):
        return f"var widget{self.idx} = new FieldPose('widget{self.idx}', '{self.name}')\n"

    def getTopicSubscriptionStrings(self):
        retStr = ""
        retStr += f'"{self.x_pos}",'
        retStr += f'"{self.y_pos}",'
        retStr += f'"{self.t_pos}",'
        return retStr

    def getJSSetData(self):
        retStr = ""
        retStr += f'if(name == "{self.x_pos}") {{\n'
        retStr += f"    widget{self.idx}.setVal(2, 0, value)\n"
        retStr += f"}}\n"
        retStr += f'if(name == "{self.y_pos}") {{\n'
        retStr += f"    widget{self.idx}.setVal(2, 1, value)\n"
        retStr += f"}}\n"
        retStr += f'if(name == "{self.t_pos}") {{\n'
        retStr += f"    widget{self.idx}.setVal(2, 2, value)\n"
        retStr += f"}}\n"
        return retStr

    def getJSUpdate(self):
        return f"    widget{self.idx}.render()"
