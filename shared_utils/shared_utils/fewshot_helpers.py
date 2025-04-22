from langchain_core.prompts import FewShotPromptTemplate, PromptTemplate


def escape_curly_braces(text):
    """
    Escape curly braces in the text by doubling them.
    """
    return text.replace("{", "{{").replace("}", "}}")


def queries_to_query_list(example):
    query_list = []
    for k,v in example.items():
        if 'query' in k.lower():
            query_list.append(v)
    updated_dict = {k: v for k,v in example.items() if 'query' not in k.lower()}
    updated_dict['queries'] = query_list
    return updated_dict


def prepare_few_shot_prompt(instructions, suffix, examples, example_variables, example_template, input_variables):
    few_shot_prompt = FewShotPromptTemplate(
        input_variables=input_variables,
        examples=examples,
        example_prompt=PromptTemplate(
            input_variables=example_variables,
            template=example_template
        ),
        prefix=instructions,
        suffix=suffix
        )
    return few_shot_prompt