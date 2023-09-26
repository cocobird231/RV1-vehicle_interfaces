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

def ConnToService(client, timeout_s, retry : int = 5):
    print("[ConnToService] Connect to service: %s (%d)\n" %(client.srv_name, retry))
    if (retry > 0):
        cnt = retry
        while (not client.wait_for_service(timeout_sec=timeout_s) and cnt > 0):
            cnt -= 1
            print('[ConnToService (%s)] Service not available, waiting again... (%d)' %(client.srv_name, cnt))
        
        if (cnt < 0):
            print('[ConnToService (%s)] Connect to service failed.' %client.srv_name)
            return False

        print('[ConnToService (%s)] Service connected.' %client.srv_name)
        return True
    else:
        while (not client.wait_for_service(timeout_sec=timeout_s)):
            print('[ConnToService (%s)] Service not available, waiting again...' %client.srv_name)

        print('[ConnToService (%s)] Service connected.' %client.srv_name)
        return True
