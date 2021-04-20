from std_msgs.msg import Float32
msg = Float32()

import importlib
msgPkg='std_msgs.msg'
msgType='Float64'
msgAttrList=['data']
mod = __import__(msgPkg,fromlist=[msgType])
msg = getattr(mod,msgType)
tmp = msg()
tmp.data = 3.14
val = getattr(tmp,msgAttrList[0])
print val

msgPkg = 'geometry_msgs.msg'
msgType='Twist'
msgAttrList=['linear','x']
mod = __import__(msgPkg,fromlist=[msgType])
msg = getattr(mod,msgType)
tmp = msg()
tmp.linear.x = 1.0

val = tmp
for attr in msgAttrList:
    val = getattr(val,attr)
print val


import functools

def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition('.')
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)

#sentinel = object()
def rgetattr(obj, attr, default=sentinel):
    if default is sentinel:
        _getattr = getattr
    else:
        def _getattr(obj, name):
            return getattr(obj, name, default)
    return functools.reduce(_getattr, [obj]+attr.split('.'))


rsetattr(tmp,'linear.x',2.0)
print tmp.linear.x
x = rgetattr(tmp,'linear.x')
print x
