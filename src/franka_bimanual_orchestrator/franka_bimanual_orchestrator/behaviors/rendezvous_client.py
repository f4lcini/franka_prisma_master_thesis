import py_trees
import rclpy
from std_srvs.srv import Trigger

class RendezvousClient(py_trees.behaviour.Behaviour):
    """
    Rendezvous node that synchronizes two arms using a unified ROS 2 Service.
    It calls /sync_arms and blocks until BOTH arms have arrived.
    """
    def __init__(self, name="Rendezvous", role="any"):
        super().__init__(name=name)
        self.node = None
        self.client = None
        self.future = None
        self.service_name = "/sync_arms"
        
    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node not found in setup kwargs!")
            return False

        self.client = self.node.create_client(Trigger, self.service_name)
        self.logger.info(f"[{self.name}] Waiting for service {self.service_name}...")
        self.client.wait_for_service(timeout_sec=1.0)
        return True

    def initialise(self):
        self.logger.info(f"[{self.name}] Reached synchronization point. Waiting for partner...")
        self.future = self.client.call_async(Trigger.Request())

    def update(self):
        if self.future is None:
            return py_trees.common.Status.FAILURE

        if self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    self.logger.info(f"✅ [{self.name}] Sync SUCCESSFUL.")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.logger.error(f"❌ [{self.name}] Sync FAILED: {response.message}")
                    return py_trees.common.Status.FAILURE
            except Exception as e:
                self.logger.error(f"❌ [{self.name}] Service call error: {str(e)}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.future = None
