def is_a_function(relation_tuples: set, domain: set, codomain: set) -> bool:
    """
    Checks if a relation (set of tuples) is a valid function f: A -> B.

    Conditions checked:
    1. Uniqueness: Each domain element maps to at most one codomain element.
    2. Total: Every domain element must be in the relation (i.e., mapped).
    """

    # 1. Check Uniqueness (The "Vertical Line Test" for sets)
    # Extract all inputs (first elements of the tuples)
    inputs_in_relation = {x for x, y in relation_tuples}

    # If the number of tuples is greater than the number of unique inputs,
    # then some input must map to multiple outputs (i.e., not unique).
    if len(relation_tuples) > len(inputs_in_relation):
        return False  # Fails the uniqueness test

    # 2. Check Totality (Every domain element is used)
    # The set of inputs used in the relation must exactly match the given domain.
    if inputs_in_relation != domain:
        return False  # Fails the totality test

    # An optional check (but good practice): ensure outputs are in the codomain
    outputs = {y for x, y in relation_tuples}
    if not outputs.issubset(codomain):
        return False  # Outputs must be elements of the codomain

    return True


def is_injective(function_tuples: set, domain: set) -> bool:
    """Checks if a function represented by a set of (input, output) tuples is injective."""
    # Extract all outputs (second elements of the tuples)
    outputs = {y for x, y in function_tuples}

    # If len(outputs) == len(domain), every domain element maps to a unique output.
    return len(outputs) == len(domain)


def is_surjective(function_tuples: set, codomain: set) -> bool:
    """Checks if a function is surjective (onto)."""
    # Extract all outputs
    outputs = {y for x, y in function_tuples}

    # Check if the set of outputs is equal to the codomain.
    return outputs == codomain


def is_bijective(function_tuples: set, domain: set, codomain: set) -> bool:
    """Checks if a function is bijective."""
    return is_injective(function_tuples, domain) and is_surjective(
        function_tuples, codomain
    )


def get_inverse_function(
    function_tuples: set, domain: set, codomain: set
) -> set | None:
    """
    Returns the inverse function (set of tuples) if the function is bijective,
    otherwise returns None.
    """
    if not is_bijective(function_tuples, domain, codomain):
        # Only bijective functions are invertible.
        return None

    # Create the inverse by swapping the elements in each tuple (y, x)
    inverse_function = {(y, x) for x, y in function_tuples}
    return inverse_function


if __name__ == "__main__":
    domain_1 = {1, 2, 3}
    codomain_1 = {"a", "b", "c"}
    function_1 = {(1, "b"), (1, "c"), (2, "a"), (3, "b")}

    domain_2 = {
        1,
        2,
        3,
    }
    codomain_2 = {"a", "b", "c"}
    function_2 = {(1, "c"), (2, "b"), (3, "a")}

    domain_3 = {1, 2}
    codomain_3 = {1, 2, 3}
    function_3 = {(1, 1), (1, 3), (2, 1), (2, 2)}

    domain_4 = {1, 2, 3}
    codomain_4 = {"a", "b", "c"}
    function_4 = {(1, "c"), (2, "c"), (3, "a")}

    inputs = (
        ("R1", domain_1, codomain_1, function_1),
        ("R2", domain_2, codomain_2, function_2),
        ("R3", domain_3, codomain_3, function_3),
        ("R4", domain_4, codomain_4, function_4),
    )

    # --- Validation and Output ---
    for i in inputs:
        name = i[0]
        domain = i[1]
        codomain = i[2]
        func = i[3]
        print(f"--- {name} Analysis ---")
        if is_a_function(func, domain, codomain):
            print(f"Injective: {is_injective(func, domain)}")  # True
            print(f"Surjective: {is_surjective(func, codomain)}")  # True
            print(f"Bijective: {is_bijective(func, domain, codomain)}")  # True
            inverse = get_inverse_function(func, domain, codomain)
            print(f"Invertible: {inverse is not None}")  # True
            if inverse:
                print(f"Inverse Function: {inverse}")  # {('b', 1), ('c', 2), ('a', 3)}
        else:
            print(f"{name} is not a function")
