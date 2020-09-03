from data_handling.wrs_persistence import WRSPersistence
from i40.loaders.cell_loader import CellLoader
from transform3d import Transform

persistence = WRSPersistence.driver()
cell = CellLoader.load(persistence)

table_t_base_b = Transform.from_xyz_rotvec(cell.find_serial_device("robot_b").world_t_base())
table_t_base_b.save('table_t_base_b')