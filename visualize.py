import utils.loader as loader

visualizer = loader.load_vis("Config/vis.yaml")
visualizer.run(["groundtrack", "att_lvlh_anim", "eci_anim"])