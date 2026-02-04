from falcor import *

def render_graph_CoopVecTestPass():
    g = RenderGraph('CoopVecTestPass')
    coop_vec_test = createPass('CoopVecTestPass')
    g.addPass(coop_vec_test, 'CoopVecTestPass')
    g.markOutput('CoopVecTestPass.output')
    return g

CoopVecTestPass = render_graph_CoopVecTestPass()
try: m.addGraph(CoopVecTestPass)
except NameError: None
