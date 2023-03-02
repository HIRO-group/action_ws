import pandas as pd
import os
import copy
import seaborn as sns
import matplotlib.pyplot as plt
import numpy as np
import plotly.express as px
from plotly.subplots import make_subplots
from plotly.graph_objs import *
from plotly import tools


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

    def find_mean(self):
        mean_vec = []
        self.df.drop(self.df[self.df['total_depth'] == 0].index, inplace=True)
        for column_name in self.df:
            col_obj = self.df[column_name]
            # print('Column Name : ', column_name)
            # print('Column Mean : ', col_obj.mean()*1000.0)
            mean_vec.append(col_obj.mean()*1000.0)
        mean_arr = np.array(mean_vec)
        np.set_printoptions(precision=1)
        np.set_printoptions(suppress=True)
        np.set_printoptions(linewidth=np.inf)
        print(mean_arr)
        return mean_arr


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

    def plot2(self, df1, df2, df3) -> None:
        # df = df.drop(columns=['total_depth'])
        # df = df.drop(columns=['panda_link0'])

        # layout = Layout(
        #     paper_bgcolor='rgba(0,0,0,0)',
        #     plot_bgcolor='rgba(0,0,0,0)'
        # )

        fig = tools.make_subplots(
            rows=1, cols=3, shared_yaxes=True, horizontal_spacing=0.01)

        figures = [
            px.line(df1, x='state_num', y=['1', '2', '3',
                    '4', '5', '6', '7', '8'],

                                        color_discrete_sequence=[
                "red", "brown", "#34cdf9", "#636EFA", "#f56b00", "grey", "mediumorchid", "#34ff34"]
            )
            # ,
            # px.line(df2, x='state_num', y=['1', '2', '3',
            #         '4', '5', '6', '7', '8'],
            #         color_discrete_sequence=[
            #     "red", "brown", "#34cdf9", "#636EFA", "#f56b00", "grey", "mediumorchid", "#34ff34"]
            # ),
            # px.line(df3, x='state_num', y=['1', '2', '3',
            #         '4', '5', '6', '7', '8'],
            #         color_discrete_sequence=[
            #     "red", "brown", "#34cdf9", "#636EFA", "#f56b00", "grey", "mediumorchid", "#34ff34"]
            # )

        ]

        for i, figure in enumerate(figures):
            for trace in range(len(figure["data"])):
                fig.append_trace(figure["data"][trace],
                                 row=1, col=1+i)

        fig['layout']['xaxis2']['title'] = 'Path Point'

        fig.update_traces(line=dict(width=12))

        fig.update_xaxes(
            title_standoff=0,
            range=[0, 600])

        fig.update_yaxes(
            range=[0, 0.13],
            title_standoff=0)

        fig.update_xaxes(showline=True, linewidth=5, linecolor='black')
        fig.update_yaxes(showline=True, linewidth=5, linecolor='black')

        fig.update_layout(
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='rgba(0,0,0,0)',
            font_family="Courier New",
            legend_title="Link",
            font_color="black",
            legend=dict(
                orientation="h"),
            # showlegend=False,
            # title_font_color="red",
            # legend_title_text='Planner',
            xaxis=dict(
                tickmode='linear',
                tick0=100,
                dtick=200,
            ),
            xaxis2=dict(
                tickmode='linear',
                tick0=100,
                dtick=200,
            ),
            xaxis3=dict(
                tickmode='linear',
                tick0=100,
                dtick=200,
            ),
            font=dict(
                family="Times New Roman",
                size=80),
            yaxis_title="Contact Depth (m)"
        )

        fig.show()


# sns.set(font_scale=1.2)
# sns.set_style(style='white')
# # sns.despine()

# reader = Reader()
# directory = "Scene4-CAT-RRT"

# arr = np.zeros(shape=(100, 11))
# i = 0
# for filename in os.listdir(directory):
#     f = os.path.join(directory, filename)
#     # checking if it is a file

#     if os.path.isfile(f) and "Traj" in f:
#         print(f)
#     else:
#         continue
#     reader.read('', f)
#     extractor = Extractor()
#     extractor.set_df(reader.df)
#     mean = extractor.find_mean()
#     arr[i] = mean
#     i = i+1

# arr = arr[~np.all(arr == 0, axis=1)]
# final_mean = np.mean(arr, axis=0)
# print("final mean")
# print(final_mean)


file_name1 = "RRTstar_FieldMagnitude_OBST_4_GOAL_1_Traj.csv"
file_name2 = "BITstar_FieldMagnitude_OBST_4_GOAL_1_Traj.csv"
file_name3 = "ContactTRRTDuo_FieldAlign_OBST_4_GOAL_1_Traj.csv"

reader1 = Reader()
reader2 = Reader()
reader3 = Reader()

reader1.read('', file_name1)
reader2.read('', file_name2)
reader3.read('', file_name3)

plotter = Plotter()
plotter.plot2(reader1.df, reader2.df, reader3.df)
