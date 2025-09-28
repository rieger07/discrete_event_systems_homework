from itertools import product


def find_domain_and_range(relation):
    """
    Identifies the domain and range of a relation.

    Args:
        relation: A set of tuples representing the relation.

    Returns:
        A tuple containing two sets: (domain, range).
    """
    # Use a set comprehension to efficiently extract unique elements
    domain = {item[0] for item in relation}
    range_set = {item[1] for item in relation}

    return domain, range_set


def is_reflexive(domain_set, relation):
    """
    Checks if a relation is reflexive on a given set.

    A relation R on a set A is reflexive if for every element a in A,
    the ordered pair (a, a) is in R.

    Args:
      domain_set: A set of elements on which the relation is defined.
      relation: A set of tuples representing the ordered pairs of the relation.

    Returns:
      True if the relation is reflexive, False otherwise.
    """
    for element in domain_set:
        if (element, element) not in relation:
            return False
    return True


def is_symmetric(relation):
    """
    Checks if a relation is symmetric.

    Args:
      relation: A set of tuples representing the ordered pairs of the relation.

    Returns:
      True if the relation is symmetric, False otherwise.
    """
    for a, b in relation:
        if (b, a) not in relation:
            return False
    return True


def is_transitive(relation):
    """
    Checks if a relation is transitive.

    Args:
      relation: A set of tuples representing the ordered pairs of the relation.

    Returns:
      True if the relation is transitive, False otherwise.
    """
    # Create a set of all unique elements involved in the relation
    elements = set(x for pair in relation for x in pair)

    for a in elements:
        for b in elements:
            for c in elements:
                # Check if (a, b) and (b, c) are in the relation
                if (a, b) in relation and (b, c) in relation:
                    # If they are, check if (a, c) is also in the relation
                    if (a, c) not in relation:
                        return False
    return True


def complement(universal_set, relation):
    """
    Computes the complement of a relation on a given domain set.

    Args:
      domain_set: The set on which the relation is defined (set A).
      relation: The set of tuples representing the relation (R).

    Returns:
      A set representing the complement of the relation (A x A - R).
    """

    # 2. Compute the set difference to find the complement
    complement_relation = universal_set - relation

    return complement_relation


if __name__ == "__main__":
    A = {1, 2, 3}
    B = {"a", "b", "c"}
    C = {1, 2}
    D = {"a", "b"}

    # AXB
    R1_domain_set = set(product(A, B))
    R1 = {(1, "b"), (1, "c"), (2, "a"), (3, "b")}

    # AXB
    R2_domain_set = set(product(A, B))
    R2 = {(1, "c"), (2, "b"), (3, "a")}

    # CXA
    R3_domain_set = set(product(C, A))
    R3 = {(1, 1), (1, 3), (2, 1), (2, 2)}

    # CXC
    R4_domain_set = set(product(C, C))
    R4 = {(1, 1), (2, 1), (2, 2)}

    # AXA
    R5_domain_set = set(product(A, A))
    R5 = {(1, 1), (1, 2), (1, 3), (2, 1), (2, 2), (3, 3)}

    # DXD
    R6_domain_set = set(product(D, D))
    R6 = {("a", "b"), ("b", "a")}

    # BXB
    R7_domain_set = set(product(B, B))
    R7 = {("a", "a"), ("a", "b"), ("a", "c"), ("b", "a"), ("b", "c"), ("c", "c")}

    # Identify the domain and range of:
    print(f"R1 domain, range: {find_domain_and_range(R1)}")
    print(f"R2 domain, range: {find_domain_and_range(R2)}")
    print(f"R3 domain, range: {find_domain_and_range(R3)}")

    # Compute the following
    # (A x B) - R1
    print(f"(A x B) -  R1 = {set(product(A,B)) - R1}")
    # (C x A) - R3
    print(f"(C x A) - R3 = {set(product(C,A)) - R3}")
    # (B x D) - R4
    print(f"(B x D) - R4 = {set(product(B,D)) - R4}")

    # Answer the Following:
    # Which of R4, R5, R6, and R7 are reflexive
    print(f"R4 is reflexive? {is_reflexive(C, R4)}")
    print(f"R5 is reflexive? {is_reflexive(A, R5)}")
    print(f"R6 is reflexive? {is_reflexive(D, R6)}")
    print(f"R7 is reflexive? {is_reflexive(B, R7)}")

    # Which of R4-7 are symmetric
    print(f"R4 is symmetric? {is_symmetric(R4)}")
    print(f"R5 is symmetric? {is_symmetric(R5)}")
    print(f"R6 is symmetric? {is_symmetric(R6)}")
    print(f"R7 is symmetric? {is_symmetric(R7)}")

    # Which of R4-7 are transitive
    print(f"R4 is transitive? {is_transitive(R4)}")
    print(f"R5 is transitive? {is_transitive(R5)}")
    print(f"R6 is transitive? {is_transitive(R6)}")
    print(f"R7 is transitive? {is_transitive(R7)}")

    # Complement of R3 wrt CxA
    print(f"(C x A) - R3 = {complement(R3_domain_set, R3)}")
    # Complement of R2 wrt AxB
    print(f"(A x B) - R2 = {complement(R2_domain_set, R2)}")
