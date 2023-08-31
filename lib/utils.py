class TopicNames():
    def __init__(self, topicName : str, ns : str = ""):
        ret = self.getFullNames(topicName, ns)
        self.ns = ret["ns"]
        self.topicName = ret["topicName"]
        self.fullName = ret["fullName"]
        self.regName = topicName
    
    @staticmethod
    def getFullNames(topicName : str, ns : str = ""):
        assert(len(topicName) > 0, "Topic name error")
        ret = dict()
        ret["topicName"] = '/' + topicName if (topicName[0] != '/') else topicName# /topicName
        ret["fullName"] = ret["topicName"]# /topicName
        if (len(ns) > 0):# Add namespace into fullName
            ret["ns"] = '/' + ns if (ns[0] != '/') else ns# /ns
            if (len(ret["ns"]) > 1 and ret["topicName"].find(ret["ns"]) != 0):# Namespace exists but not included in topicName
                ret["fullName"] = ret["ns"] + ret["topicName"]# /ns/topicName
        else:
            ret["ns"] = ""
        return ret
    
    def __eq__(self, obj):
        return self.fullName == obj.fullName