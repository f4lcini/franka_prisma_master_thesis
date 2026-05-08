import py_trees
import rclpy
from franka_custom_interfaces.srv import HandoverReady

class RendezvousClient(py_trees.behaviour.Behaviour):
    """
    Rendezvous node that synchronizes two arms using a ROS 2 Service.
    It calls /donor_ready or /recipient_ready and blocks until the coordinator releases it.
    """
    def __init__(self, name="Rendezvous", role="donor"):
        super().__init__(name=name)
        self.role = role # "donor" or "recipient"
        self.node = None
        self.client = None
        self.future = None
        self.service_name = f"/{self.role}_ready"
        
    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
        except KeyError:
            self.logger.error(f"[{self.name}] ROS 2 node not found in setup kwargs!")
            return False

        self.client = self.node.create_client(HandoverReady, self.service_name)
        self.logger.info(f"[{self.name}] Waiting for service {self.service_name}...")
        self.client.wait_for_service(timeout_sec=1.0)
        return True

    def initialise(self):
        self.logger.info(f"[{self.name}] Initiating Rendezvous as {self.role} on {self.service_name}...")
        
        request = HandoverReady.Request()
        request.arm_id = self.role
        request.timeout_sec = 120.0
        
        self.future = self.client.call_async(request)

    def update(self):
        if self.future is None:
            return py_trees.common.Status.FAILURE

        if self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    self.logger.info(f"✅ [{self.name}] Rendezvous SUCCESSFUL: {response.message}")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.logger.error(f"❌ [{self.name}] Rendezvous FAILED: {response.message}")
                    return py_trees.common.Status.FAILURE
            except Exception as e:
                self.logger.error(f"❌ [{self.name}] Service call error: {str(e)}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.future = None
