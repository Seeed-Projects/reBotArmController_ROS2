"""Client lifecycle checks with mocked ROS and signal boundaries."""
import signal
from unittest.mock import Mock, patch

from rebotarmcontroller.examples import gravity_compensation as gc


def run_client(*, service_ready=True, heartbeat_timeout=False, interrupt=False):
    node=Mock()
    clients={}
    def client(_type,name):
        clients[name]=Mock()
        clients[name].service_is_ready.return_value=service_ready
        return clients[name]
    node.create_client.side_effect=client
    handlers={}
    def register(sig,handler):
        handlers[sig]=handler
    def spin(*_args,**_kwargs):
        if interrupt:
            handlers[signal.SIGINT](None,None)
    with patch.object(gc,'Node',return_value=node), \
         patch.object(gc,'_call_trigger',return_value=True) as calls, \
         patch.object(gc.rclpy,'init'), \
         patch.object(gc.rclpy,'shutdown'), \
         patch.object(gc.rclpy,'ok',side_effect=[True,True,False]), \
         patch.object(gc.rclpy,'spin_once',side_effect=spin), \
         patch.object(gc.signal,'getsignal'), \
         patch.object(gc.signal,'signal',side_effect=register), \
         patch.object(gc.time,'monotonic',side_effect=[0.,0.,4.,4.] if heartbeat_timeout else [0.,0.,.1,.2]):
        gc.main()
    node.destroy_node.assert_called_once()
    return [call.args[2] for call in calls.call_args_list]


def test_controller_service_disappearance_exits_without_cleanup_calls():
    assert run_client(service_ready=False)==['enable','start gravity compensation']


def test_heartbeat_timeout_exits_even_when_services_remain_visible():
    assert run_client(heartbeat_timeout=True)==['enable','start gravity compensation']


def test_client_ctrl_c_preserves_home_then_disable_cleanup():
    assert run_client(interrupt=True)==['enable','start gravity compensation','safe_home','disable']
