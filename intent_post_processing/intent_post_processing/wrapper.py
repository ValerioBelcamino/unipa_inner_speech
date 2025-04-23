import inspect


def make_callable_from_context(func):
    sig = inspect.signature(func)
    param_names = list(sig.parameters.keys())

    def wrapper(context):
        kwargs = {name: context[name] for name in param_names if name in context}
        return func(**kwargs)

    wrapper.__name__ = f"call_{func.__name__}"
    return wrapper