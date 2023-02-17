import pandas as pd
import os
import copy
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np


class Reader:

    path = ""
    file = []
    df = pd.DataFrame()

    def __init__(self) -> None:
        pass

    def read(self, path: str, file: str) -> None:
        self.path = path
        self.file = file

        f_path = os.path.join(self.path, self.file)
        df = pd.read_csv(f_path)
        df.name = f_path
        self.df = df

    def get_df(self) -> pd.DataFrame:
        return copy.deepcopy(self.df)

    def set_df(self, df: pd.DataFrame) -> None:
        self.df = df


sns.set(font_scale=2)

reader = Reader()
path = ""
file_name = "ContactTRRTDuo_FieldAlign_OBST_3_GOAL_1.csv"
reader.read(path, file_name)

for column_name in reader.df:
    column_obj = reader.df[column_name]
    print('Column Name : ', column_name)
    print('Column Contents : ', column_obj.values)
    print('Column Mean : ', column_obj.describe())
