import sqlite3

db_path = "/home/srge/.ros/tests/Go2_small_rock_t4/Go2_small_rock_t4.db"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
tables = cursor.fetchall()

print("Tables found in the database:")
for table in tables:
    print(table[0])

conn.close()
