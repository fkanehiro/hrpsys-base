import rtm

global mgr, rtc1, rtc2

# Managerへの参照を取得します
mgr = rtm.findRTCmanager()
# NullComponent.soをロードします
mgr.load("NullComponent")
# NullComponentのインスタンスを2つ作ります
rtc1 = mgr.create("NullComponent", "np1")
rtc2 = mgr.create("NullComponent", "np2")
# 2つのコンポーネントを接続します
rtm.connectPorts(rtc1.port("dataOut"), rtc2.port("dataIn"))
rtcs = [rtc1, rtc2]
# 2つのコンポーネントが同期実行されるように設定します
rtm.serializeComponents(rtcs)
# 2つのコンポーネントをアクティベートします
rtc1.start()
rtc2.start()

