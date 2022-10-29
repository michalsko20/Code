from tkinter import *
from tkinter.ttk import *
import snake1 as fl
import os
import sqlite3


def start_game():
    os.startfile("snake1.py")


def connect():

    con1 = sqlite3.connect("score.db")
    cur1 = con1.cursor()
    cur1.execute("CREATE TABLE IF NOT EXISTS wynik(id INTEGER PRIMARY KEY, username TEXT, score TEXT)")
    con1.commit()
    con1.close()


def show_score_board():
    global tree
    connect()
    root = Tk()
    tree = Treeview(root, column=("c1", "c2"), show='headings')
    tree.column("#1", anchor=CENTER)
    tree.heading("#1", text="username")
    tree.column("#2", anchor=CENTER)
    tree.heading("#2", text="score")
    tree.pack()
    con1 = sqlite3.connect("score.db")
    cur1 = con1.cursor()
    cur1.execute("SELECT * FROM wynik ORDER BY score DESC")
    rows = cur1.fetchall()
    i=0
    for row in rows:
        if i ==8:
            break
        print(row)
        tree.insert("", END, values=row)
        i += 1
    con1.close()
    root.mainloop()

def Close():
    master.destroy()

master = Tk()

master.geometry("250x250")

label = Label(master, text="Menu")
label.pack(side=TOP, pady=10)
btn = Button(master,
             text="Start game")

btn.bind("<Button>",
         lambda e: start_game())
btn.pack(pady=10)

score_button = Button(master, text="Scoreboard", command=lambda: show_score_board())
score_button.pack(pady=20)
exit_button = Button(master, text="Exit", command=Close)
exit_button.pack(pady=20)

mainloop()