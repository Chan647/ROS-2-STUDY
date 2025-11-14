import pymysql

DB_CONFIG = dict(
    host ="localhost",
    user ="root",
    password="qwe!@#456",
    database="rosdb",
    charset="utf8"
)

class DB :
    def __init__(self,**config):
        self.config = config

    def connect(self):
        return pymysql.connect(**self.config)
    
    def insert_con(self, X, Y, THETA, TIME):
        sql = "INSERT INTO turtlepos (X, Y, THETA, TIME) VALUES (%s, %s, %s, %s)"
        with self.connect() as conn:
            try:
                with conn.cursor() as cur:
                    cur.execute(sql, (X, Y, THETA, TIME))
                conn.commit()
                return True
            except Exception:
                conn.rollback()
                return False