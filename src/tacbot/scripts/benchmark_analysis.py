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


class Extractor:
    df = pd.DataFrame()

    def ___init__(self) -> None:
        pass

    def set_df(self, df: pd.DataFrame) -> None:
        self.df = df

    def get_df(self) -> pd.DataFrame:
        return copy.deepcopy(self.df)

    def find_mean(self) -> None:
        self.df.drop(self.df[self.df['success'] == 0].index, inplace=True)
        for column_name in self.df:
            col_obj = self.df[column_name]
            print('Column Name : ', column_name)
            print('Column Mean : ', col_obj.mean())


reader = Reader()
path = "Scene4-CAT-RRT"  # Scene4-CAT-RRT"
#file_name = "RRTstar_FieldMagnitude_OBST_4_GOAL_1.csv"
#file_name = "BITstar_FieldMagnitude_OBST_4_GOAL_1.csv"
file_name = "ContactTRRTDuo_FieldAlign_OBST_4_GOAL_1.csv"

reader.read(path, file_name)

extractor = Extractor()
extractor.set_df(reader.df)
extractor.find_mean()
