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


class Extractor:
    df = pd.DataFrame()

    def ___init__(self) -> None:
        pass

    def set_df(self, df: pd.DataFrame) -> None:
        self.df = df

    def get_df(self) -> pd.DataFrame:
        return copy.deepcopy(self.df)

    def find_mean(self) -> None:
        mean_vec = []
        for column_name in self.df:
            col_obj = self.df[column_name]
            print('Column Name : ', column_name)
            print('Column Mean : ', col_obj.mean()*1000.0)
            mean_vec.append(col_obj.mean()*1000.0)
        mean_arr = np.array(mean_vec)
        np.set_printoptions(precision=1)
        np.set_printoptions(suppress=True)
        np.set_printoptions(linewidth=np.inf)
        print(mean_arr)


class Plotter:

    fig_size = [3, 5]
    y_label = ""

    def __init__(self) -> None:
        self.init_figure()

    def init_figure(self) -> None:
        self.fig, self.ax = plt.subplots(
            figsize=(self.fig_size[0], self.fig_size[1]))

    def clear_ax(self) -> None:
        self.ax.cla()

    def summary(self, df: pd.DataFrame) -> None:
        print(df.describe())

    def plot(self, df: pd.DataFrame) -> None:
        df = df.drop(columns=['total_depth'])
        df = df.drop(columns=['panda_link0'])
        self.ax.set_xlim(0, 800)

        # sns.lineplot(data=df[['panda_link1','panda_link2','panda_link3','panda_link4','panda_link5','panda_link6','panda_link7','panda_link8']])
        plt1 = sns.lineplot(x='state_num', y='value', hue='variable', linewidth=4,
                            data=pd.melt(df, ['state_num']))
        self.ax.set(xlabel='Sample Number', ylabel='Penetration Depth (m)')
        self.ax.get_legend().remove()
        self.fig.set_tight_layout(True)
        # plt.legend(title='Link', loc='upper right',
        #            labels=['1', '2', '3', '4', '5', '6', '7', 'EE'])
        # plt1.set(ylabel=None)
        # plt1.set(yticklabels=[])
        # plt.savefig('scene4-rrtstar.png')
        # plt.show()


sns.set(font_scale=1.2)
sns.set_style(style='white')
# sns.despine()

reader = Reader()
path = ""
#file_name = "RRTstar_FieldMagnitude_OBST_5_GOAL_1_Traj.csv"
#file_name = "BITstar_FieldMagnitude_OBST_5_GOAL_1_Traj.csv"
file_name = "ContactTRRTDuo_FieldAlign_OBST_5_GOAL_1_Traj.csv"

reader.read(path, file_name)

extractor = Extractor()
extractor.set_df(reader.df)
extractor.find_mean()
