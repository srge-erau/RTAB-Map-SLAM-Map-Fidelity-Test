import sqlite3

db_path = "/home/srge/.ros/tests/Go2_small_rock_t4/Go2_small_rock_t4.db"
conn = sqlite3.connect(db_path)
cursor = conn.cursor()

# List columns in Node table
cursor.execute("PRAGMA table_info(Node);")
columns = cursor.fetchall()
print("\nColumns in Node table:")
for col in columns:
    print(col)

# List columns in Data table
cursor.execute("PRAGMA table_info(Data);")
columns = cursor.fetchall()
print("\nColumns in Data table:")
for col in columns:
    print(col)

# List columns in Feature table
cursor.execute("PRAGMA table_info(Feature);")
columns = cursor.fetchall()
print("\nColumns in Feature table:")
for col in columns:
    print(col)

# List columns in Link table
cursor.execute("PRAGMA table_info(Link);")
columns = cursor.fetchall()
print("\nColumns in Link table:")
for col in columns:
    print(col)

# List columns in Info table
cursor.execute("PRAGMA table_info(Info);")
columns = cursor.fetchall()
print("\nColumns in Info table:")
for col in columns:
    print(col) 


conn.close()
