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


class Cleaner:
    df = pd.DataFrame()

    def ___init__(self) -> None:
        pass

    def set_df(self, df: pd.DataFrame) -> None:
        self.df = df

    def get_df(self) -> pd.DataFrame:
        return copy.deepcopy(self.df)

    def remove_outliers(self) -> None:
        print("Outliers removed")


class Plotter:

    fig_size = [20, 15]
    y_label = ""

    def __init__(self) -> None:
        self.init_figure()

    def init_figure(self) -> None:
        self.fig, self.ax = plt.subplots(
            figsize=(self.fig_size[0], self.fig_size[1]))

    def clear_ax(self) -> None:
        self.ax.cla()

    def plot(self, df: pd.DataFrame) -> None:
        df = df.drop(columns=['total_depth'])
        # sns.lineplot(data=df[['panda_link1','panda_link2','panda_link3','panda_link4','panda_link5','panda_link6','panda_link7','panda_link8']])
        sns.lineplot(x='state_num', y='value', hue='variable',
                     data=pd.melt(df, ['state_num']))
        plt.show()


sns.set(font_scale=2)
sns.set_style(style='white')

reader = Reader()
path = ""
file_name = "BITstar_FieldMagnitude_OBST_3_GOAL_1_Traj.csv"
reader.read(path, file_name)


plotter = Plotter()
plotter.plot(reader.df)
