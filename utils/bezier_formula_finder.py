"""
Calcul des coefficients polynomiaux d'une courbe de Bézier de degré n.
Exprime P(t) = Sum(i=0..n) a_i * t^i, où les a_i sont des combinaisons
linéaires des points de contrôle P_0, ..., P_n.
"""

import sympy as sp


def bezier_coefficients(n: int):
    t = sp.Symbol('t')

    # Points de contrôle symboliques : p0, p1, ..., pn
    P = [sp.Symbol(f'p{i}') for i in range(n + 1)]

    # ── Formule de Bézier générale ──────────────────────────────────────────
    # B(t) = Sum_{k=0}^{n} C(n,k) * t^k * (1-t)^(n-k) * P_k
    B = sum(sp.binomial(n, k) * t**k * (1 - t)**(n - k) * P[k]
            for k in range(n + 1))

    B_expanded = sp.expand(B)

    # ── Extraction des coefficients par puissance de t ──────────────────────
    poly = sp.Poly(B_expanded, t)

    print(f"\n{'='*55}")
    print(f"  Courbe de Bézier de degré n = {n}")
    print(f"{'='*55}")
    print(f"\n  Formule : P_{n}(t) = Sum(i=0, {n}): a_i * t^i\n")
    print(f"  Coefficients :")
    print(f"  {'-'*45}")

    for i in range(n, -1, -1):
        coeff = poly.nth(i)
        # Factorisation légère pour un affichage plus lisible
        coeff_factored = sp.factor(coeff)
        # On garde la forme la plus courte entre développée et factorisée
        coeff_display = (coeff_factored
                         if len(str(coeff_factored)) <= len(str(coeff))
                         else coeff)
        print(f"  a_{i} = {coeff_display}")

    print(f"\n  Expression complète développée :")
    # Affichage avec les puissances de t en ordre décroissant
    terms = []
    for i in range(n, -1, -1):
        coeff = poly.nth(i)
        if coeff != 0:
            if i == 0:
                terms.append(f"({coeff})")
            elif i == 1:
                terms.append(f"({coeff})*t")
            else:
                terms.append(f"({coeff})*t^{i}")
    print(f"  P_{n}(t) = " + " + ".join(terms))
    print(f"{'='*55}\n")


# ── Point d'entrée ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        n = int(input("Entrez le degré n de la courbe de Bézier : "))
        if n < 1:
            print("Le degré doit être >= 1.")
        else:
            bezier_coefficients(n)
    except ValueError:
        print("Veuillez entrer un entier valide.")
