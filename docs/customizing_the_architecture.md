![Architecture](../archi.jpg)

# 🧩 Custom Scenario Integration Guide

This system is designed to support multiple scenarios, allowing for the definition and integration of custom use cases, each with dedicated tools, databases, and operations.

A scenario defines:

- A general description of the task/domain
- One or more databases used in operations
- A set of supported operations (tools)

## 📁 Directory Structure

To extend the system with a custom scenario, place your content inside the `scenario_customization` package using the following structure:

```
scenario_customization/
└── MY_CUSTOM_SCENARIO/
    ├── scenario_description.txt
    ├── plugin_config.yaml
    ├── MyOperation1/
    │   ├── intent.py
    │   ├── query.py
    │   └── examples/
    │       ├── explainability_examples.json
    │       └── query_examples.json
    └── MyOperation2/
        └── ...
```

## 📘 scenario_description.txt

This is a plain text file that provides a natural language description of the scenario. It is included in prompts during inner speech, intent recognition, and explainability.

**Example:**

```
You are an AI assistant that has to support a user in managing their personalized diet and nutrition.
```

## ⚙️ plugin_config.yaml

Used to configure optional intent post-processing plugins loaded dynamically. These are implemented using the plugin design pattern.

**Example:**

```yaml
plugins:
  - name: "param_to_lower"
    module: "intent_post_processing.plugins.db_convention_plugin"
    function: "param_to_lower"
    
  - name: "check_user_weekly_plan"
    module: "intent_post_processing.plugins.db_convention_plugin"
    function: "check_user_weekly_plan"

  - name: "get_day_of_the_week"
    module: "intent_post_processing.plugins.time_plugin"
    function: "get_day_of_the_week"
    
  - name: "get_next_meal"
    module: "intent_post_processing.plugins.time_plugin"
    function: "get_next_meal"
```

## 🛠 Operation Folders

Each operation is modeled as a tool and placed in a subfolder inside the scenario folder.

Each tool folder must contain:

### 1. intent.py

Defines the parameters the user is expected to provide. Based on `pydantic.BaseModel`, with descriptions, types, and optional default values.

### 2. query.py

Specifies one or more queries to be executed. Can return a single query or a list of queries with their respective priorities.

If the operation needs a different DB than the default scenario DB (e.g., Qdrant instead of SQLite), define it via the private `_DB` field:

```python
class MovieInfoTool(BaseModel):
    """Returns a query to fetch movie info from a vector store based on the user input."""
    query: str = Field(description="Query to Qdrant vector db to fetch most similar movies")
    _DB: str = 'qdrant'
```

### 3. examples/ Folder

Contains few-shot examples for explainability and query generation modules. These improve model performance via prompting.

#### explainability_examples.json

**Format:**

```json
[
  {
    "user_input": "Puoi inserirmi nel sistema? Sono Franco e ho un fabbisogno di 2500kcal...",
    "queries": "MATCH (r:Recipe {name: 'panna cotta'}) OPTIONAL MATCH ...",
    "results": "[{'p': {...}}, {'a': {...}}]",
    "explanation": "La panna cotta non contiene glutine..."
  },
  ...
]
```

#### query_examples.json

**Format:**

```json
[
  {
    "question": "Ciao, sono Luca. Vorrei essere inserito nel sistema con 2000 calorie...",
    "parameters": "{'nome_utente': 'luca', 'calorie': '2000', ...}",
    "query": "CREATE (p:Person {name: 'luca', calories: 2000, ...}) RETURN p;"
  },
  ...
]
```

## 🛢 Supported Databases

Scenarios and operations can specify the use of:

- `sqlite`
- `neo4j`
- `qdrant`

If an operation needs a non-default DB, use `_DB` in the operation's query model.

New DBs can be added by extending:

- `db_adapters/db_adapter.py`: Define the new adapter class
- `db_adapters/db_factory.py`: Implement its instantiation logic

These modules are structured using the Adapter and Factory design patterns for modularity.

## 🔧 System Configuration

Models used in different modules (e.g., LLMs for inner speech or query generation) can be configured in `.config` files found in the root directory of each module.