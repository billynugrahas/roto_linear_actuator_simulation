import xlsxwriter
import do_mpc
import numpy as np


i = '001'

with_prediction = False

# Create a workbook and add a worksheet.
if not with_prediction:
    workbook = xlsxwriter.Workbook(f'./excel/{i}_spring_mass.xlsx')
else:
    workbook = xlsxwriter.Workbook(f'./excel/{i}_spring_mass_pred.xlsx')

worksheet = workbook.add_worksheet()

# Some data we want to write to the worksheet.
results = do_mpc.data.load_results(f'./results/{i}_spring_mass.pkl')

n_loop = results['mpc']['_u'].shape[0]
u = results['mpc']['_u']
set_point = results['mpc']['_tvp']
x = results['mpc']['_x']

n_horizon =  results['mpc'].__dict__['meta_data']['n_horizon']
t_step =  results['mpc'].__dict__['meta_data']['t_step']
r_term = 1e-2

excel_arr = np.concatenate((u, set_point, x), axis = 1)

# breakpoint()

# Start from the first cell. Rows and columns are zero indexed.
row = 1
col = 0

worksheet.write(0, col, 'MPC Params')
mpc_params = (
    ['n_horizon', n_horizon], 
    ['t_step', t_step], 
    ['r_term', r_term]
)

# Iterate over the data and write it out row by row.
for param, value in (mpc_params):
    worksheet.write(row, col,     param)
    worksheet.write(row, col + 1, value)
    row += 1

row += 1
worksheet.write(row, col, 'MPC Data')
row += 1
worksheet.write(row, col, 'Time (s)')
worksheet.write(row, col + 1, 'Input (N)')
worksheet.write(row, col + 2, 'Position Set Point (m)')
worksheet.write(row, col + 3, 'Speed Set Point (m/s)')
worksheet.write(row, col + 4, 'Position (m)')
worksheet.write(row, col + 5, 'Speed (m/s)')
row += 1

for index, (input, x_set_point, x_dot_set_point, state_x, state_x_dot) in enumerate(excel_arr):
    worksheet.write(row, col,     index * t_step)
    worksheet.write(row, col + 1, input)
    worksheet.write(row, col + 2, x_set_point)
    worksheet.write(row, col + 3, x_dot_set_point)
    worksheet.write(row, col + 4, state_x)
    worksheet.write(row, col + 5, state_x_dot)
    
    pred_x = results['mpc'].prediction(('_x', 'x'), t_ind=index)[0]
    pred_x_dot = results['mpc'].prediction(('_x', 'x'), t_ind=index)[0]

    if with_prediction:
        row += 1
        for pred_index, x in enumerate(pred_x):
            worksheet.write(row, col+4, x)
            worksheet.write(row, col+5, pred_x_dot[pred_index])
            row += 1
    else:
        row += 1


workbook.close()