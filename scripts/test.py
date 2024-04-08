a = {1: "data", 1023: "teta"}

try:
    print(a[22])
except Exception as e:
    # print(f"Get Error Exception: {e}")
    if(isinstance(e, KeyError)):
        print(f"{e.args[0]}")
        # print(f"error: {type(e).__name__}")