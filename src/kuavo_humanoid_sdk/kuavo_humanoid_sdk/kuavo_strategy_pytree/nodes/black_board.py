import py_trees


class BlackBoardManager:
    """
    一些白板的管理函数
    """

    @staticmethod
    def erase_namespace(ns: str):
        """删掉 ns.* 的所有键"""
        board = py_trees.blackboard.Blackboard()
        keys = [k for k in list(board.storage.keys()) if k.startswith(ns + ".")]
        for k in keys:
            board.unset(k)

    @staticmethod
    def print_blackboard():
        board = py_trees.blackboard.Blackboard()
        print("==== Blackboard contents ====")
        for k, v in board.storage.items():
            print(f"{k}: {v}")
        print("==== End ====")
