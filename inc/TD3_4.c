#include <stdio.h>
#include <stdlib.h>
#include <math.h>

void afficher (int T[3][3]);
void saisir(int T[3][3], int num_joueur);

int tester_ligne(int T[3][3], int ligne);
int tester_colonne(int T[3][3], int col);

int tester_diagonale1(int T[3][3]);
int tester_diagonale2(int T[3][3]);
int tester_grille(int T[3][3]);

int main()
{
	int tab[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
	afficher(tab);
	saisir(tab,2);
	afficher(tab);
	
}

void afficher(int T[3][3])
{
	int i,j;
	for (i =0; i< 3;i++)
	{
		for (j = 0; j<3; j++)
			printf("%3d",T[i][j]);
			
		printf("\n");
	}
}

void saisir(int T[3][3], int num_joueur)
{
	int flag = 0;
	int l,col;
	while (flag != 1)
	{
		printf("Choisisser une ligne : \n");
		scanf("%d",&l);
		printf("Choisisser une colone : \n");
		scanf("%d",&col);
		if (T[l][col] == 0)
		{
			T[l][col] = num_joueur;
			flag = 1;
		}
	}
}

int tester_ligne(int T[3][3], int ligne)
{
	if (T[ligne][0] == T[ligne][1] &&  T[ligne][2] == T[ligne][1])
		return T[ligne][0];
	else
		return 0;
}

int tester_colonne(int T[3][3], int col)
{
	if (T[0][col] == T[1][col] &&  T[2][col] == T[1][col])
		return T[0][col];
	else
		return 0;
}

int tester_diagonale1(int T[3][3])
{
	if(T[0][0] == T[1][1] && T[2][2] == T[1][1])
		return T[0][0];
	else
		return 0;
}

int tester_diagonale2(int T[3][3])
{
	if(T[0][2] == T[1][1] && T[2][0] == T[1][1])
		return T[0][2];
	else
		return 0;
}

int tester_grille(int T[3][3])
{
	int l,c,d1,d2;
	for (int i = 0;i<3; i++)
	{
		l = tester_ligne(T,i);
		c = tester_colonne(T,i);
	}
	d1 = tester_diagonale1(T);
	d2 = tester_diagonale1(T);
	if (l != 0 || c != 0 || d1 != 0 || d2 != 0)
		return;
}
