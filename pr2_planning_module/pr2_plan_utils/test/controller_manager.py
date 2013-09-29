import roslib; roslib.load_manifest('pr2_plan_utils')
import sys
from pr2_plan_utils.controller_manager_client import ControllerManagerClient

controllers_to_start = [sys.argv[1]]
controllers_to_stop = []

cmc = ControllerManagerClient()
print 'Controllers:'
print cmc.list_controllers()
cmc.switch_controllers(controllers_to_start, controllers_to_stop)
print ''
print 'Controllers now:'
print cmc.list_controllers()

