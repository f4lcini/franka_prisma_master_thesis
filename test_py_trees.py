import py_trees
from py_trees.display import unicode_tree
b = py_trees.behaviours.Success(name="MySuc")
b.tick_once()
b.stop(py_trees.common.Status.INVALID)
print(unicode_tree(b, show_status=True))
