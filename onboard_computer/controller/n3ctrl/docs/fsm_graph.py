import graphviz as gv
from graphviz import Digraph
import subprocess

g = gv.Digraph(format='pdf')
g.attr('graph', compound='true')

n3_graph = Digraph('n3_graph')
n3_graph.attr('graph', packMode='graph')

ctrl_graph = Digraph('cluster_ctrl')
ctrl_graph.attr('graph', packMode='graph')

node_style = dict(
    shape='ellipse',
    style='filled'
    )

node_err_style = dict(
    shape='ellipse',
    style='filled',
    fillcolor='yellow'
    )

node_legend_style = dict(
    shape = 'box'
    )

js_graph = Digraph('js')

cmd_graph = Digraph('cmd')

legend_g = Digraph('cluster_legend')
legend_g.attr('graph', 
    color = 'dimgray',
    style = 'solid',
    label = 'LEGEND',
    fontname = 'Courier',
    rankdir = 'LR',
    orientation = '90'
)

n3_graph.node('JS_DIRECT', 'Joystick direct control', **node_style)
n3_graph.node('JS_CTRL_RAW', 'Joystick control without odom', **node_style)

js_graph.node('JS_CTRL', 'Joystick control with odom', **node_style)
js_graph.node('JS_RESET_POS', 'Reset pos for js ctrl', **node_err_style)
js_graph.node('JS_LOCK_Z_0_RP', 'Lock z, r=p=0 for js ctrl', **node_err_style)

cmd_graph.node('CMD_HOVER', 'Hover for cmd ctrl', **node_style)
cmd_graph.node('CMD_CTRL', 'Command control', **node_style)
cmd_graph.node('CMD_RESET_POS', 'Reset pos for cmd ctrl', **node_err_style)
cmd_graph.node('CMD_LOCK_Z_0_RP', 'Lock z, r=p=0 for cmd ctrl', **node_err_style)

legend_g.node('EX_LOW_LEVEL', 'Low level ctrl', **node_style)
legend_g.node('EX_HIGH_LEVEL', 'High level ctrl', **node_style)
legend_g.node('EX_LC', 'Loop-closure', **node_err_style)
legend_g.node('EX_ERR', 'Error', **node_err_style)

# error
edge_style = dict(
    color="red",
    )
ctrl_graph.edge('CMD_CTRL', 'CMD_HOVER', 'cmd-x\n|cmd-too-far\n|collision', **edge_style)
ctrl_graph.edge('CMD_HOVER', 'CMD_LOCK_Z_0_RP', 'odom-x', **edge_style)
ctrl_graph.edge('CMD_CTRL', 'CMD_LOCK_Z_0_RP', 'odom-x', **edge_style)
ctrl_graph.edge('CMD_RESET_POS', 'CMD_LOCK_Z_0_RP', 'odom-x', **edge_style)
ctrl_graph.edge('JS_CTRL', 'JS_LOCK_Z_0_RP', 'odom-x', **edge_style)
ctrl_graph.edge('JS_RESET_POS', 'JS_LOCK_Z_0_RP', 'odom-x', **edge_style)
legend_g.edge('EX_HIGH_LEVEL','EX_ERR', 'error(lost)', **edge_style)
legend_g.edge('EX_LC','EX_ERR', 'error(lost)', **edge_style)

# half-error
edge_style = dict(
    color="red",
    style="dashed"
    )
ctrl_graph.edge('CMD_CTRL', 'CMD_RESET_POS', 'odom-o', **edge_style)
ctrl_graph.edge('CMD_HOVER', 'CMD_RESET_POS', 'odom-o', **edge_style)
ctrl_graph.edge('JS_CTRL', 'JS_RESET_POS', 'odom-o', **edge_style)
legend_g.edge('EX_HIGH_LEVEL','EX_LC', 'loop-closure', **edge_style)

# recovery
edge_style = dict(
    color="green",
    )
ctrl_graph.edge('CMD_RESET_POS', 'CMD_HOVER', 'odom-v', **edge_style)
ctrl_graph.edge('CMD_LOCK_Z_0_RP', 'CMD_HOVER', 'odom-v', **edge_style)
ctrl_graph.edge('JS_LOCK_Z_0_RP', 'JS_CTRL', 'odom-v', **edge_style)
ctrl_graph.edge('JS_RESET_POS', 'JS_CTRL', 'odom-v', **edge_style)
legend_g.edge('EX_ERR','EX_HIGH_LEVEL', 'recovery', **edge_style)
legend_g.edge('EX_LC','EX_HIGH_LEVEL', 'recovery', **edge_style)


# half-recovery
edge_style = dict(
    color="green",
    style="dashed"
    )
ctrl_graph.edge('CMD_LOCK_Z_0_RP', 'CMD_RESET_POS', 'odom-o', **edge_style)
ctrl_graph.edge('JS_LOCK_Z_0_RP', 'JS_RESET_POS', 'odom-o', **edge_style)
legend_g.edge('EX_ERR','EX_LC', 'half-recovery', **edge_style)

# go to higher level
edge_style = dict(
    color="blue"
    )
g.edge('JS_DIRECT', 'JS_CTRL', 'rc-api    ', **edge_style)
g.edge('JS_DIRECT', 'JS_CTRL_RAW', 'rc-api && raw-mode', **edge_style)
ctrl_graph.edge('JS_CTRL', 'CMD_HOVER', 'sw-to-cmd', **edge_style)
ctrl_graph.edge('CMD_HOVER', 'CMD_CTRL', 'cmd-v\n& cmd-r', **edge_style)
legend_g.edge('EX_LOW_LEVEL','EX_HIGH_LEVEL', 'go to higher level', **edge_style)


# switch return lower level
edge_style = dict(
    color="purple"
    )
g.edge('JS_CTRL', 'JS_DIRECT', ' rc-api-off', ltail='cluster_ctrl', **edge_style)
g.edge('JS_CTRL_RAW', 'JS_DIRECT', ' rc-api-off', **edge_style)
ctrl_graph.edge('CMD_HOVER', 'JS_CTRL', 'sw-to-js', **edge_style)
ctrl_graph.edge('CMD_CTRL', 'JS_CTRL', 'sw-to-js', **edge_style)
ctrl_graph.edge('CMD_HOVER', 'JS_DIRECT', 'js-act / rc-api-off', **edge_style)
ctrl_graph.edge('CMD_CTRL', 'JS_DIRECT', 'js-act / rc-api-off', **edge_style)
legend_g.edge('EX_HIGH_LEVEL','EX_LOW_LEVEL', 'go to lower level', **edge_style)


subgraph_color = 'dimgray'
graph_style = dict(
    color = subgraph_color,
    style = 'dashed',
    label = "sub-states for joystick control"
    )
g_rc_ctrl = gv.Digraph('cluster_rc_ctrl')
g_rc_ctrl.attr('graph', **graph_style)
g_rc_ctrls = []
sublabels = ['X', 'Y', 'Z']
for i in range(3):
    sg = gv.Digraph('cluster_' + str(i))
    if i == 0:
        sg.node('Fix', 'Fix')
        sg.node('Move', 'Move')
        sg.node('Break', 'Break')
        sg.edge('Fix', 'Move', 'Joystick operated')
        sg.edge('Move', 'Break', 'Joystick released')
        sg.edge('Break', 'Fix', 'Speed down')
    else:
        sg.node('JS_AXIS_OMIT'+str(i), 'same as %s'%sublabels[0], shape='folder')
    sg.body.append('label="axis %s"'%sublabels[i])
    sg.body.append('color=%s'%subgraph_color)
    sg.body.append('style=solid')
    g_rc_ctrls.append(sg)

for i in range(3):
    g_rc_ctrl.subgraph(g_rc_ctrls[i])
    edge_style = dict(color='%s'%subgraph_color, dir='none', style='dashed')
    if i==0:
        ctrl_graph.edge('JS_CTRL','Fix', **edge_style)
    else:
        ctrl_graph.edge('JS_CTRL','JS_AXIS_OMIT'+str(i), **edge_style)

node_style = dict(
    shape='box',
    fontname='Courier',
    style='solid'
    )

width_l = 11
width_r = 25
help_string = [
("sw-to-cmd","switch to cmd mode by RC"),
("sw-to-js","switch to js mode by RC"),
("rc-api-on","RC in API mode"),
("rc-api-off","RC not in API mode"),
("js-act","User acts the joystick"),
("odom-v","odometry valid"),
("odom-x","odometry invalid"),
("odom-o","odometry loop-closure"),
("cmd-v","command valid"),
("cmd-x","command invalid"),
("cmd-r","command renewed"),
]

help_string = '\n'.join(
    ["HELP".center(width_l+width_r),] 
    + [s[0].ljust(width_l)+": "+s[1].ljust(width_r) for s in help_string])
g.node("HELP", help_string, **node_style)

js_graph.subgraph(g_rc_ctrl)

ctrl_graph.subgraph(js_graph)
ctrl_graph.subgraph(cmd_graph)

g.subgraph(n3_graph)
g.subgraph(ctrl_graph)
g.subgraph(legend_g)

g.render('fsm')
subprocess.Popen(['gnome-open', 'fsm.pdf'])
