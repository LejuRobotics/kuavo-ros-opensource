FACTORY_URL = "git://10.11.99.175:9418/kuavo-ros-opensource.git"
GITEE_URL = "https://gitee.com/leju-robot/kuavo-ros-opensource.git"
KUAVO_ROS_ALLOWED_URLS = (FACTORY_URL, GITEE_URL)

SOURCE_MODE_AUTO = "auto"
SOURCE_MODE_FACTORY = "factory"
SOURCE_MODE_GITEE = "gitee"


def is_allowed_remote(remote_url, allowed_urls):
    remote = (remote_url or "").strip()
    return any(remote == url for url in allowed_urls)


def expected_urls_text(allowed_urls):
    return " 或 ".join(allowed_urls)


def clone_sources(mode=SOURCE_MODE_AUTO):
    if mode == SOURCE_MODE_FACTORY:
        return [FACTORY_URL]
    if mode == SOURCE_MODE_GITEE:
        return [GITEE_URL]
    return [FACTORY_URL, GITEE_URL]


def branch_exists(ls_remote_output, branch_name):
    return f"refs/heads/{branch_name}" in (ls_remote_output or "")


def commit_exists_in_origin_factory(branch_contains_output):
    return any("origin_factory/" in line for line in (branch_contains_output or "").splitlines())
