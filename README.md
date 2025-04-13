# easyEVO-GUI

Existing features:

python FrontEnd.py --testing
python FrontEnd.py --continue
python FrontEnd.py --reset

Testing:
- Plots output_test.csv file in the interface. 
- Assumes no serial connection.
- Useful if you have an easyEvo csv file and want a quick and dirty plot

Continue and Reset are similar. They both
- Connect to the machine with a wire
- Run and it will create a live serial connection through which you can pull the csv file onto your computer

The difference between Continue and Reset:
- Continue: Assumes that you want to continue your current run, and maintains those stats
- Reset: Establishes a serial connection, but then start a new run

Features to add:
- Make the view on top and the info/control panels on the bottom
- Be able to merge experiments in the csv file