/*
 *  myfs.c - Implementacao do sistema de arquivos MyFS
 *
 *  Autores: Débora Izabel Rocha Duarte
 *           Michel  Gomes de Andrade
 *  Projeto: Trabalho Pratico - Sistemas Operacionais
 *  Semestre: 2023-3
 *  Professor: Marcelo Moreno
 *  Organizacao: Universidade Federal de Juiz de Fora
 *  Departamento: Dep. Ciencia da Computacao
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "myfs.h"
#include "vfs.h"
#include "inode.h"
#include "util.h"

#define SB_FIRST_BLOCK_SECTOR (2 * sizeof(unsigned int) + sizeof(char))
#define SB_NUM_BLOCKS (3 * sizeof(unsigned int) + sizeof(char))
#define SB_FREE_SPACE_SECTOR (sizeof(unsigned int) + sizeof(char))
#define SB_BLOCKSIZE 0

typedef struct
{
	char filename[100];
	unsigned int inumber;
} LinkDir;

typedef struct // estrutura para arquivo
{
	Disk *disk;
	Inode *inode;
	unsigned int blocksize;
	unsigned int lastByteRead;
	const char *path;
	unsigned int fd;
} File;

typedef struct // estrutura para diretório
{
	Disk *disk;
	Inode *inode;
	unsigned int blocksize;
	unsigned int lastByteRead;
	const char *path;
	unsigned int fd;
} Directory;

typedef struct // estrutura para entrada diretório
{
	char filename[MAX_FILENAME_LENGTH];
	unsigned int inumber;
} DirectoryEntry;

// Declaracoes globais
FSInfo *fsInfo;
File *files[MAX_FDS] = {NULL};
Directory *directories[MAX_FDS] = {NULL};

// Auxiliares

/* Responsável por encontrar a posição do primeiro bit de valor 0 em um byte. */
int firstZeroBit(unsigned char byte)
{
	unsigned char mask = 1; // Inicializa uma máscara com o bit menos significativo (LSB) definido
	int i;					// Variável de controle do loop

	for (i = 0; i < sizeof(unsigned char); i++)
	{
		if ((mask & byte) == 0) // Verifica se o bit atual é 0
			return i;			// Retorna a posição do primeiro bit de valor 0

		mask <<= (unsigned char)1; // Desloca a máscara para a próxima posição de bit
	}

	return -1;
}

/*Responsável por definir o bit de uma posição específica para 1 em um byte.*/
unsigned char setBitToOne(unsigned char byte, unsigned int bit)
{
	unsigned char mask = (unsigned char)1 << bit; // Cria uma máscara com o bit desejado definido como 1
	return byte | mask;							  // Define o bit para 1 usando a operação OR
}

/*Responsável por definir o bit de uma posição específica para 0 em um byte. */
unsigned char setBitToZero(unsigned char byte, unsigned int bit)
{
	unsigned char mask = ((unsigned char)1 << bit); // Cria uma máscara com o bit desejado definido como 1
	mask = ~mask;									// Inverte a máscara para ter 0 no bit desejado e 1 nos outros
	return byte & mask;								// Define o bit para 0 usando a operação AND
}

/*Responsável por encontrar um bloco livre no sistema de arquivos. */
unsigned int findFreeBlock(Disk *disk)
{
	unsigned char buffer[DISK_SECTORDATASIZE]; // Declara um buffer para armazenar dados de um setor do disco.
	if (diskReadSector(disk, 0, buffer) == -1) // Lê o setor 0 para obter informações do superbloco. Retorna -1 em caso de falha.
		return -1;

	unsigned int sectorsPerBlock; // Obtém o número de setores por bloco do superbloco.
	char2ul(&buffer[SB_BLOCKSIZE], &sectorsPerBlock);
	sectorsPerBlock /= DISK_SECTORDATASIZE;

	unsigned int numBlocks; // Obtém o número total de blocos do superbloco.
	char2ul(&buffer[SB_NUM_BLOCKS], &numBlocks);

	unsigned int firstBlock; // Obtém o setor do primeiro bloco do superbloco.
	char2ul(&buffer[SB_FIRST_BLOCK_SECTOR], &firstBlock);

	unsigned int freeSpaceSector; // Obtém o setor de espaço livre do superbloco.
	char2ul(&buffer[SB_FREE_SPACE_SECTOR], &freeSpaceSector);

	unsigned int freeSpaceSize = firstBlock - freeSpaceSector; // Calcula o tamanho do espaço livre em setores.

	for (int i = freeSpaceSector; i < freeSpaceSector + freeSpaceSize; i++)
	{											   // Itera sobre os setores de espaço livre.
		if (diskReadSector(disk, i, buffer) == -1) // Lê um setor de espaço livre. Retorna -1 em caso de falha.
			return -1;

		for (int j = 0; j < DISK_SECTORDATASIZE; j++)
		{										   // Itera sobre os bytes do setor de espaço livre.
			int freeBit = firstZeroBit(buffer[j]); // Encontra o primeiro bit livre no byte.

			if (freeBit != -1)
			{ // Verifica se um bit livre foi encontrado.
				unsigned int freeBlock = firstBlock +
										 (i - freeSpaceSector) * DISK_SECTORDATASIZE * 8 * sectorsPerBlock +
										 j * 8 * sectorsPerBlock +
										 freeBit * sectorsPerBlock; // Calcula o número do bloco livre com base no bit livre encontrado.

				if ((freeBlock - firstBlock) / sectorsPerBlock >= numBlocks)
					return -1; // Verifica se o bloco livre está dentro do limite de blocos.

				buffer[j] = setBitToOne(buffer[j], freeBit); // Define o bit correspondente como ocupado no byte.
				if (diskWriteSector(disk, i, buffer) == -1)	 // Escreve o setor de espaço livre modificado de volta no disco.
					return -1;

				return freeBlock; // Retorna -1 se nenhum bloco livre for encontrado.
			}
		}
	}

	return -1; // Retorna o número do bloco livre encontrado.
}

/*Responsável por marcar um bloco como livre no sistema de arquivos.*/
bool setBlockFree(Disk *d, unsigned int block)
{
	unsigned char buffer[DISK_SECTORDATASIZE]; // Declara um buffer para armazenar dados de um setor do disco.
	if (diskReadSector(d, 0, buffer) == -1)	   // Lê o setor 0 para obter informações do superbloco. Retorna false em caso de falha.
		return false;

	unsigned int sectorsPerBlock;
	char2ul(&buffer[SB_BLOCKSIZE], &sectorsPerBlock);
	sectorsPerBlock /= DISK_SECTORDATASIZE; // Obtém o número de setores por bloco do superbloco.

	unsigned int numBlocks;
	char2ul(&buffer[SB_NUM_BLOCKS], &numBlocks); // Obtém o número total de blocos do superbloco.

	unsigned int firstBlock;
	char2ul(&buffer[SB_FIRST_BLOCK_SECTOR], &firstBlock); // Obtém o setor do primeiro bloco do superbloco.

	unsigned int freeSpaceStartSector;
	char2ul(&buffer[SB_FREE_SPACE_SECTOR], &freeSpaceStartSector); // Obtém o setor de início do espaço livre do superbloco.

	if ((block - firstBlock) / sectorsPerBlock >= numBlocks)
		return false; // Verifica se o bloco está dentro dos limites do sistema de arquivos. Retorna false se estiver fora dos limites.

	unsigned int blockFreeSpaceSector = ((block - firstBlock) / sectorsPerBlock) / (DISK_SECTORDATASIZE * 8); // Calcula o setor no qual as informações do espaço livre do bloco são armazenadas.
	if (diskReadSector(d, blockFreeSpaceSector, buffer) == -1)
		return false; // Lê o setor do espaço livre do bloco. Retorna false em caso de falha.

	unsigned int blockFreeSpaceBit = ((block - firstBlock) / sectorsPerBlock) % (DISK_SECTORDATASIZE * 8); // Calcula o bit correspondente ao bloco no setor do espaço livre.
	buffer[blockFreeSpaceBit / 8] = setBitToZero(buffer[blockFreeSpaceBit / 8], blockFreeSpaceBit % 8);	   // Marca o bit correspondente ao bloco como livre no byte do setor do espaço livre.
	if (diskWriteSector(d, blockFreeSpaceSector, buffer) == -1)
		return false; // Escreve o setor do espaço livre do bloco modificado de volta no disco.

	return true; // Retorna true se a operação for bem-sucedida, ou seja, se o bloco foi marcado como livre com sucesso.
}

/*Responsável por buscar um arquivo no sistema de arquivos com base no disco e caminho do arquivo.*/
File *getFile(Disk *d, const char *path)
{
	// Inicia um loop para percorrer todas as entradas do array de arquivos (files) até o número máximo de descritores de arquivo (MAX_FDS).
	for (int i = 0; i < MAX_FDS; i++)
	{
		if (files[i] != NULL &&				   // Verifica se a entrada do array não é nula
			files[i]->disk == d &&			   // se o disco associado à entrada é igual ao disco passado como parâmetro
			strcmp(files[i]->path, path) == 0) // e se o caminho do arquivo associado à entrada é igual ao caminho passado como parâmetro.
		{
			return files[i]; // retorna a entrada do array de arquivos correspondente ao arquivo encontrado.
		}
	}
	// Se nenhum arquivo correspondente for encontrado no loop, retorna NULL para indicar que o arquivo não está presente no sistema de arquivos.
	return NULL;
}

// Do professor

// Funcao para verificacao se o sistema de arquivos está ocioso, ou seja,
// se nao ha quisquer descritores de arquivos em uso atualmente. Retorna
// um positivo se ocioso ou, caso contrario, 0.
int myFSIsIdle(Disk *d)
{
	// loop para percorrer todas as entradas do array de arquivos até o número máximo de descritores de arquivo (MAX_FDS).
	for (int i = 0; i < MAX_FDS; i++)
	{
		// Verifica se a entrada do array não é nula e se o ID do disco passado como parâmetro é igual ao ID do disco associado à entrada.
		if (files[i] != NULL &&						   // Garante que a entrada do array não está vazia.
			diskGetId(d) == diskGetId(files[i]->disk)) // Compara os IDs dos discos para verificar se são iguais.
		{
			return 0; // sistema de arquivos não está ocioso
		}
	}
	return 1; // sem arquivo no loop sistema de arquivos está ocioso
}

// Funcao para formatacao de um disco com o novo sistema de arquivos
// com tamanho de blocos igual a blockSize. Retorna o numero total de
// blocos disponiveis no disco, se formatado com sucesso. Caso contrario,
// retorna -1.
int myFSFormat(Disk *d, unsigned int blockSize)
{
	unsigned char superblock[DISK_SECTORDATASIZE] = {0}; // Inicializa um array de bytes para armazenar o superbloco (superblock) com zeros.

	ul2char(blockSize, &superblock[SB_BLOCKSIZE]); // funçao para conversao de um unsigned int para um array de bytes, poe no lugar certo

	unsigned int numInodes = (diskGetSize(d) / blockSize) / 8; // Calcula o número de inodes com base no tamanho do disco e no tamanho do bloco.

	unsigned int freeSpaceSector = inodeAreaBeginSector() + numInodes / inodeNumInodesPerSector();				   // Calcula o setor de espaço livre com base no início da área de inodes.
	unsigned int freeSpaceSize = (diskGetSize(d) / blockSize) / (sizeof(unsigned char) * 8 * DISK_SECTORDATASIZE); // Calcula o tamanho do espaço livre em setores.

	ul2char(freeSpaceSector, &superblock[SB_FREE_SPACE_SECTOR]); // transforma o setor de espaço livre em um array de bytes

	unsigned int firstBlockSector = freeSpaceSector + freeSpaceSize;										// Calcula o setor do primeiro bloco com base no setor de espaço livre e no tamanho do espaço livre.
	unsigned int numBlocks = (diskGetNumSectors(d) - firstBlockSector) / (blockSize / DISK_SECTORDATASIZE); // Calcula o número total de blocos com base no número total de setores disponíveis.

	ul2char(firstBlockSector, &superblock[SB_FIRST_BLOCK_SECTOR]); // setor do primeiro bloco em array de bytes
	ul2char(numBlocks, &superblock[SB_NUM_BLOCKS]);				   // numero total de blocos em array de bytes

	if (diskWriteSector(d, 0, superblock) == -1) // Escreve o superbloco no primeiro setor do disco.
		return -1;								 // Se houver um erro na escrita, retorna -1.

	unsigned char freeSpace[DISK_SECTORDATASIZE] = {0}; // cria array de bytes para representar espaço libre

	for (int i = 0; i < freeSpaceSize; i++) // escrever nos setores de espaço livre
	{
		if (diskWriteSector(d, freeSpaceSector + i, freeSpace) == -1) // Escreve cada setor de espaço livre no disco
		{
			return -1; // Se houver um erro na escrita, retorna -1.
		}
	}

	return numBlocks > 0 ? numBlocks : -1; // Retorna o número total de blocos se for maior que 0, caso contrário, retorna -1.
}

/*Encontra e retorna um ponteiro para a estrutura File associada aocaminho (path) e o disco (d).*/
File *MygetFile(Disk *d, const char *path)
{
	for (int i = 0; i < MAX_FDS; i++) // percorre todas as entradas no array files
	{
		if (files[i] != NULL &&				   // Verifica se a entrada atual no array files não é nula
			files[i]->disk == d &&			   // se o disco associado à entrada é o mesmo que o disco fornecido (d)
			strcmp(files[i]->path, path) == 0) // se o caminho (path) associado à entrada é o mesmo que o caminho fornecido.
		{
			return files[i]; // retorna o ponteiro para a estrutura File
		}
	}
	return NULL; // retorna NULL, indicando que nenhum arquivo foi encontrado para o caminho fornecido no disco especificado.
}

// Funcao para abertura de um arquivo, a partir do caminho especificado
// em path, no disco montado especificado em d, no modo Read/Write,
// criando o arquivo se nao existir. Retorna um descritor de arquivo,
// em caso de sucesso. Retorna -1, caso contrario.
int myFSOpen(Disk *d, const char *path)
{
	File *file = getFile(d, path); // ponteiro para o arquivo com base no caminho fornecido
	int numInode;				   // armazena o número do inode do arquivo

	// se arquivo não existe vai criar o arquivo
	if (file == NULL)
	{
		Inode *inode = NULL;
		for (int i = 0; i < MAX_FDS; i++) // percoore as entradas no array files para encontrar um slot disponível
		{
			if (files[i] == NULL) // indica se estrada atual está vazia, se sim, tem um slot disponível
			{
				numInode = inodeFindFreeInode(1, d); // Encontra um inode livre no disco (d) e armazena seu número na variável numInode.
				inode = inodeCreate(numInode, d);	 // Cria um novo inode com o número obtido no passo anterior.
				break;
			}
		}

		if (inode == NULL)
			return -1; // retorna -a se houver falha na criação do inode

		file = malloc(sizeof(File));	  // Aloca dinamicamente memória para o arquivo
		file->disk = d;					  // define o campo disk da estrutura File com o disco fornecido.
		file->path = path;				  // define o caminho
		file->inode = inode;			  // define o inode
		file->fd = inodeGetNumber(inode); // define o campo fd
		files[file->fd - 1] = file;		  // Adiciona o arquivo ao array files na posição correspondente ao seu número de inode.
	}

	return file->fd; // Retorna o descritor de arquivo (file descriptor) associado ao arquivo.
}

// Funcao para a leitura de um arquivo, a partir de um descritor de
// arquivo existente. Os dados lidos sao copiados para buf e terao
// tamanho maximo de nbytes. Retorna o numero de bytes efetivamente
// lidos em caso de sucesso ou -1, caso contrario.
int myFSRead(int fd, char *buf, unsigned int nbytes)
{
	// Verifica se o descritor de arquivo é válido. Se não for, retorna -1 indicando erro.
	if (fd < 0 || fd >= MAX_FDS)
		return -1;

	File *file = files[fd]; // pega ponteiro do arquivo associeado a esse descritor
	if (file == NULL)
		return -1; // Verifica se o arquivo existe. Se não existir, retorna -1 indicando erro.

	unsigned int fileSize = inodeGetFileSize(file->inode);							  // obtem tamanho total do arquivo
	unsigned int bytesRead = 0;														  // inicializa contador de bytes lidos
	unsigned int currentInodeBlockNum = file->lastByteRead / file->blocksize;		  // Calcula o número do bloco do inode que contém o último byte lido.
	unsigned int offset = file->lastByteRead % file->blocksize;						  // Calcula o deslocamento dentro do bloco do inode do último byte lido.
	unsigned int currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNum); // Obtém o número do bloco real no disco correspondente ao bloco do inode atual.
	unsigned char diskBuffer[DISK_SECTORDATASIZE];									  // Declara um buffer para armazenar dados lidos do disco.

	while (bytesRead < nbytes &&
		   bytesRead + file->lastByteRead < fileSize &&
		   currentBlock > 0) // loop para leitura de dados
	{
		unsigned int sectorsPerBlock = file->blocksize / DISK_SECTORDATASIZE; // nº de sotores por bloco
		unsigned int firstSector = offset / DISK_SECTORDATASIZE;			  // Calcula o índice do primeiro setor dentro do bloco real.
		unsigned int firstByteInSector = offset % DISK_SECTORDATASIZE;		  // Calcula o deslocamento dentro do primeiro setor.

		for (int i = firstSector; i < sectorsPerBlock && bytesRead < nbytes; i++) // Loop para ler dados de cada setor dentro do bloco
		{
			if (diskReadSector(file->disk, currentBlock + i, diskBuffer) == -1)
				return -1; // Lê o setor atual do disco para o buffer. Se falhar, retorna -1 indicando erro.

			for (int j = firstByteInSector; // Loop para copiar dados do buffer para o buffer de saída (buf).
				 j < DISK_SECTORDATASIZE &&
				 bytesRead < nbytes &&
				 bytesRead + file->lastByteRead < fileSize;
				 j++)
			{
				buf[bytesRead] = diskBuffer[j];
				bytesRead++;
			}

			firstByteInSector = 0; // reseta o deslocamento dentro do primeiro bloco
		}

		offset = 0;															 // reseta o deslocamento
		currentInodeBlockNum++;												 // avança para o próximo
		currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNum); // Obtém o número do bloco real correspondente ao próximo bloco do inode.
	}

	file->lastByteRead += bytesRead; // Atualiza o último byte lido na estrutura File.

	return bytesRead; // Retorna o número total de bytes lidos.
}

// Funcao para a escrita de um arquivo, a partir de um descritor de
// arquivo existente. Os dados de buf serao copiados para o disco e
// terao tamanho maximo de nbytes. Retorna o numero de bytes
// efetivamente escritos em caso de sucesso ou -1, caso contrario
int myFSWrite(int fd, const char *buf, unsigned int nbytes)
{
	if (fd <= 0 || fd > MAX_FDS)
		return -1; // Verifica se o descritor de arquivo é válido. Se não for, retorna -1 indicando erro.

	File *file = files[fd]; // ponteiro para arquivo

	if (!file)
		return -1; // Verifica se o arquivo existe. Se não existir, retorna -1 indicando erro.

	unsigned int fileSize = inodeGetFileSize(file->inode);							  // Obtém o tamanho total do arquivo associado ao File.
	unsigned int bytesWritten = 0;													  // Inicializa a variável bytesWritten para contar o número de bytes escritos.
	unsigned int currentInodeBlockNum = file->lastByteRead / file->blocksize;		  // Calcula o número do bloco do inode que contém o último byte lido.
	unsigned int offset = file->lastByteRead % file->blocksize;						  // Calcula o deslocamento dentro do bloco do inode do último byte lido.
	unsigned int currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNum); // Obtém o número do bloco real no disco correspondente ao bloco do inode atual.
	unsigned char diskBuffer[DISK_SECTORDATASIZE];									  // Declara um buffer para armazenar dados lidos do disco.

	while (bytesWritten < nbytes) // Loop principal para escrever dados
	{
		unsigned int sectorsPreBlock = file->blocksize / DISK_SECTORDATASIZE; // Calcula o número de setores por bloco.
		unsigned int firstSector = offset / DISK_SECTORDATASIZE;			  // Calcula o índice do primeiro setor dentro do bloco real.
		unsigned int firstByteInSector = offset % DISK_SECTORDATASIZE;		  // Calcula o deslocamento dentro do primeiro setor.

		// Se o bloco atual for 0, aloca um novo bloco e o adiciona ao inode.
		if (currentBlock == 0)
		{
			currentBlock = findFreeBlock(file->disk);

			if (currentBlock == -1)
				break; // Se não for possível alocar um bloco, encerra o loop.

			if (inodeAddBlock(file->inode, currentBlock) == -1)
			{
				setBlockFree(file->disk, currentBlock); // Libera o bloco se falhar em adicionar ao inode.
				break;
			}
		}

		for (int i = firstSector; i < sectorsPreBlock && bytesWritten < nbytes; i++)
		{ // Loop para escrever dados em cada setor dentro do bloco real.
			if (diskReadSector(file->disk, currentBlock + i, diskBuffer) == -1)
				return -1; // Lê o setor atual do disco para o buffer.

			for (int j = firstByteInSector; j < DISK_SECTORDATASIZE && bytesWritten < nbytes; j++)
			{ // Loop para copiar dados do buffer de entrada (buf) para o buffer do disco.
				diskBuffer[j] = buf[bytesWritten];
				bytesWritten++; // Copia um byte do buffer de entrada para o buffer do disco.
			}

			if (diskWriteSector(file->disk, currentBlock + i, diskBuffer) == -1)
				return -1; // Escreve o buffer do disco de volta no disco.

			firstByteInSector = 0; // Reseta o deslocamento dentro do primeiro setor após o primeiro setor.
		}

		// Reseta o deslocamento para 0 e avança para o próximo bloco do inode.
		offset = 0;
		currentInodeBlockNum++;
		currentBlock = inodeGetBlockAddr(file->inode, currentInodeBlockNum);
	}

	file->lastByteRead += bytesWritten; // Atualiza o último byte lido na estrutura File.
	if (file->lastByteRead >= fileSize)
	{														 // Se o último byte lido for maior ou igual ao tamanho total do arquivo
		inodeSetFileSize(file->inode, currentInodeBlockNum); // atualiza o tamanho do arquivo no inode
		inodeSave(file->inode);								 // salva no disco.
	}
	return bytesWritten; // Retorna o número total de bytes escritos.
}

// Funcao para fechar um arquivo, a partir de um descritor de arquivo
// existente. Retorna 0 caso bem sucedido, ou -1 caso contrario
int myFSClose(int fd)
{
	// Verifica se o descritor de arquivo é válido. Se não for, retorna -1 indicando erro.
	if (fd <= 0 || fd > MAX_FDS)
		return -1;

	File *file = files[fd];
	if (!file)
		return -1; // Verifica se o arquivo existe. Se não existir, retorna -1 indicando erro.

	files[fd - 1] = NULL; // Libera a entrada correspondente no array de arquivos, marcando-a como nula.
	free(file->inode);	  // Libera a memória alocada para a estrutura Inode associada ao arquivo.
	free(file);			  // Libera a memória alocada para a estrutura File.

	return 0; // Retorna 0 indicando sucesso.
}

// Funcao para abertura de um diretorio, a partir do caminho
// especificado em path, no disco indicado por d, no modo Read/Write,
// criando o diretorio se nao existir. Retorna um descritor de arquivo,
// em caso de sucesso. Retorna -1, caso contrario.
int myFSOpenDir(Disk *d, const char *path)
{
	// Verifica se o caminho é nulo. Se for, retorna -1 indicando erro.
	if (path == NULL)
		return -1;

	// Verifica se o comprimento do caminho é zero. Se for, retorna -1 indicando erro.
	if (strlen(path) == 0)
		return -1;

	Directory *dir = NULL; // Inicializa um ponteiro para a estrutura Directory.

	for (int i = 0; i < MAX_FDS; i++) // Itera sobre o array de diretórios.
	{
		if (directories[i] == NULL) // Verifica se a entrada do diretório está vazia.
		{
			dir = malloc(sizeof(Directory)); // Aloca memória para a estrutura Directory.
			if (dir == NULL)
			{
				return -1; // Verifica se a alocação de memória foi bem-sucedida. Se não for, retorna -1 indicando erro.
			}

			unsigned int inodeNumber = inodeFindFreeInode(1, d); // Encontra um número de inode livre para o diretório.
			if (inodeNumber == 0)
			{
				free(dir);
				return -1; // Verifica se a operação de encontrar inode livre foi bem-sucedida. Se não for, libera a memória alocada e retorna -1 indicando erro.
			}

			dir->inode = inodeCreate(inodeNumber, d); // Cria um novo inode para o diretório.
			if (dir->inode == NULL)
			{
				free(dir);
				return -1; // Verifica se a criação do inode foi bem-sucedida. Se não for, libera a memória alocada e retorna -1 indicando erro.
			}

			// Inicializa os campos da estrutura Directory.
			dir->disk = d;
			dir->path = path;
			dir->blocksize = SB_BLOCKSIZE;
			dir->lastByteRead = 0;

			directories[i] = dir; // Atribui o diretório recém-criado ao array de diretórios.

			return i + 1; // Retorna o descritor de diretório, que é o índice no array de diretórios + 1.
		}
	}
	return -1; // Se não for possível encontrar um descritor de diretório disponível, retorna -1 indicando erro.
}

// Função auxiliar para obter a próxima entrada de diretório
int getNextDirEntry(Disk *disk, Inode *dirInode, unsigned int *cursor, DirectoryEntry *entry)
{
	unsigned int dirSize = inodeGetFileSize(dirInode); // Obtém o tamanho do diretório, que é a soma dos tamanhos dos blocos ocupados pelo diretório.

	// Verifica se já atingiu o fim do diretório
	if (*cursor >= dirSize)
	{
		return 0; // Fim do diretório
	}

	// Calcula o número do bloco e a posição dentro do bloco em que o cursor está localizado.
	unsigned int blockNum = *cursor / DISK_SECTORDATASIZE;
	unsigned int blockOffset = *cursor % DISK_SECTORDATASIZE;

	unsigned char blockBuffer[DISK_SECTORDATASIZE]; // Cria um buffer para armazenar o conteúdo do bloco.
	// Lê o bloco do diretório
	if (diskReadSector(disk, inodeGetBlockAddr(dirInode, blockNum), blockBuffer) == -1)
	{
		return -1; // Falha na leitura
	}

	// Copia a entrada de diretório do bloco para a estrutura entry
	memcpy(entry, blockBuffer + blockOffset, sizeof(DirectoryEntry));

	// Atualiza o cursor
	*cursor += sizeof(DirectoryEntry);

	return 1; // Entrada de diretório lida com sucesso
}

// Funcao para a leitura de um diretorio, identificado por um descritor
// de arquivo existente. Os dados lidos correspondem a uma entrada de
// diretorio na posicao atual do cursor no diretorio. O nome da entrada
// e' copiado para filename, como uma string terminada em \0 (max 255+1).
// O numero do inode correspondente 'a entrada e' copiado para inumber.
// Retorna 1 se uma entrada foi lida, 0 se fim de diretorio ou -1 caso
// mal sucedido
int myFSReadDir(int fd, char *filename, unsigned int *inumber)
{
	// Verifica se o descritor de arquivo é válido
	if (fd <= 0 || fd > MAX_FDS)
	{
		return -1; // Descritor de arquivo inválido
	}

	File *dirFile = files[fd - 1]; // ponteiro para arquivo
	if (dirFile == NULL)
	{
		return -1; // Diretório não encontrado
	}

	// Obtém a próxima entrada de diretório
	DirectoryEntry entry; // estrutura para armazenar os dados de entrada no diretório

	// Obtém a próxima entrada de diretório chamando o método getNextDirEntry. O resultado da operação é armazenado em result.
	int result = getNextDirEntry(dirFile->disk, dirFile->inode, &dirFile->lastByteRead, &entry);

	if (result == 1)
	{
		// Entrada de diretório lida com sucesso, copia os dados para os parâmetros de saída
		strncpy(filename, entry.filename, MAX_FILENAME_LENGTH);
		*inumber = entry.inumber;
	}

	return result; // Retorna o resultado da leitura do diretório
}

// Funcao para adicionar uma entrada a um diretorio, identificado por um
// descritor de arquivo existente. A nova entrada tera' o nome indicado
// por filename e apontara' para o numero de i-node indicado por inumber.
// Retorna 0 caso bem sucedido, ou -1 caso contrario.
int myFSLink(int fd, const char *filename, unsigned int inumber)
{
	// Verifica se o descritor de arquivo é válido
	if (fd <= 0 || fd > MAX_FDS)
	{
		return -1; // Descritor de arquivo inválido
	}

	File *dirFile = files[fd - 1];
	if (dirFile == NULL)
	{
		return -1; // Diretório não encontrado
	}

	// Verifica se o nome do arquivo é válido
	if (strlen(filename) == 0 || strlen(filename) > MAX_FILENAME_LENGTH)
	{
		return -1; // Nome de arquivo inválido
	}

	// Cria uma entrada de diretório com os dados fornecidos
	DirectoryEntry newEntry;
	strncpy(newEntry.filename, filename, MAX_FILENAME_LENGTH);
	newEntry.inumber = inumber;

	// Calcula o número do bloco e a posição dentro do bloco
	unsigned int blockNum = dirFile->lastByteRead / DISK_SECTORDATASIZE;
	unsigned int blockOffset = dirFile->lastByteRead % DISK_SECTORDATASIZE;

	// Lê o bloco do diretório
	unsigned char blockBuffer[DISK_SECTORDATASIZE];
	if (diskReadSector(dirFile->disk, inodeGetBlockAddr(dirFile->inode, blockNum), blockBuffer) == -1)
	{
		return -1; // Falha na leitura
	}

	// Verifica se há espaço suficiente no bloco para a nova entrada
	if (blockOffset + sizeof(DirectoryEntry) > DISK_SECTORDATASIZE)
	{
		return -1; // Não há espaço suficiente no bloco
	}

	// Copia a nova entrada para o bloco do diretório
	memcpy(blockBuffer + blockOffset, &newEntry, sizeof(DirectoryEntry));

	// Escreve o bloco atualizado de volta no disco
	if (diskWriteSector(dirFile->disk, inodeGetBlockAddr(dirFile->inode, blockNum), blockBuffer) == -1)
	{
		return -1; // Falha na escrita
	}

	// Atualiza o cursor
	dirFile->lastByteRead += sizeof(DirectoryEntry);

	return 0; // Link criado com sucesso
}

// Funcao para remover uma entrada existente em um diretorio,
// identificado por um descritor de arquivo existente. A entrada e'
// identificada pelo nome indicado em filename. Retorna 0 caso bem
// sucedido, ou -1 caso contrario.
int myFSUnlink(int fd, const char *filename)
{
	// Verifica se o descritor de arquivo é válido
	if (fd <= 0 || fd > MAX_FDS)
	{
		return -1; // Descritor de arquivo inválido
	}

	File *dirFile = files[fd - 1];
	if (dirFile == NULL)
	{
		return -1; // Diretório não encontrado
	}

	// Verifica se o nome do arquivo é válido
	if (strlen(filename) == 0 || strlen(filename) > MAX_FILENAME_LENGTH)
	{
		return -1; // Nome de arquivo inválido
	}

	// Número do bloco e posição dentro do bloco
	unsigned int blockNum = dirFile->lastByteRead / DISK_SECTORDATASIZE;
	unsigned int blockOffset = dirFile->lastByteRead % DISK_SECTORDATASIZE;

	// Loop para encontrar a entrada de diretório com o nome especificado
	DirectoryEntry currentEntry;
	while (blockNum < inodeGetFileSize(dirFile->inode))
	{
		// Lê o bloco do diretório
		unsigned char blockBuffer[DISK_SECTORDATASIZE];
		if (diskReadSector(dirFile->disk, inodeGetBlockAddr(dirFile->inode, blockNum), blockBuffer) == -1)
		{
			return -1; // Falha na leitura
		}

		// Loop através das entradas de diretório no bloco
		while (blockOffset < DISK_SECTORDATASIZE)
		{
			// Copia a entrada de diretório atual
			memcpy(&currentEntry, blockBuffer + blockOffset, sizeof(DirectoryEntry));

			// Verifica se é a entrada desejada
			if (strncmp(currentEntry.filename, filename, MAX_FILENAME_LENGTH) == 0)
			{
				// Remove a entrada do bloco do diretório
				memset(blockBuffer + blockOffset, 0, sizeof(DirectoryEntry));

				// Escreve o bloco atualizado de volta no disco
				if (diskWriteSector(dirFile->disk, inodeGetBlockAddr(dirFile->inode, blockNum), blockBuffer) == -1)
				{
					return -1; // Falha na escrita
				}

				// Atualiza o cursor
				dirFile->lastByteRead = blockNum * DISK_SECTORDATASIZE + blockOffset;

				return 0; // Remoção bem-sucedida
			}

			// Avança para a próxima entrada
			blockOffset += sizeof(DirectoryEntry);
		}

		// Avança para o próximo bloco
		blockNum++;
		blockOffset = 0;
	}

	// Entrada não encontrada
	return -1;
}

// Funcao para fechar um diretorio, identificado por um descritor de
// arquivo existente. Retorna 0 caso bem sucedido, ou -1 caso contrario.
int myFSCloseDir(int fd)
{
	// Verifica se o descritor de arquivo é válido
	if (fd <= 0 || fd > MAX_FDS)
	{
		return -1; // Descritor de arquivo inválido
	}

	File *dirFile = files[fd - 1];
	if (dirFile == NULL)
	{
		return -1; // Diretório não encontrado
	}

	// Libera a estrutura de arquivo associada ao diretório
	files[fd - 1] = NULL;
	free(dirFile->inode);
	free(dirFile);

	return 0; // Fechamento bem-sucedido
}

// Funcao para instalar seu sistema de arquivos no S.O., registrando-o junto
// ao virtual FS (vfs). Retorna um identificador unico (slot), caso
// o sistema de arquivos tenha sido registrado com sucesso.
// Caso contrario, retorna -1
int installMyFS(void)
{

	// Aloca dinamicamente memória para a estrutura FSInfo e armazena o ponteiro resultante na variável global fsInfo.
	fsInfo = malloc(sizeof(FSInfo));

	// Registra o sistema de arquivos no VFS (sistema de arquivos virtual) chamando vfsRegisterFS. O ID do sistema de arquivos retornado é armazenado no campo fsid da estrutura fsInfo.
	fsInfo->fsid = (char)vfsRegisterFS(fsInfo);

	// Associa a função myFSIsIdle ao campo isidleFn da estrutura fsInfo. Esta função é utilizada para verificar se o sistema de arquivos está inativo.
	fsInfo->isidleFn = myFSIsIdle;

	// Associa a função myFSFormat ao campo formatFn da estrutura fsInfo. Esta função é utilizada para formatar o sistema de arquivos.
	fsInfo->formatFn = myFSFormat;

	// Associa a função myFSOpen ao campo openFn da estrutura fsInfo. Esta função é utilizada para abrir um arquivo no sistema de arquivos.
	fsInfo->openFn = myFSOpen;

	// Associa a função myFSRead ao campo readFn da estrutura fsInfo. Esta função é utilizada para ler dados de um arquivo no sistema de arquivos.
	fsInfo->readFn = myFSRead;

	// Associa a função myFSWrite ao campo writeFn da estrutura fsInfo. Esta função é utilizada para escrever dados em um arquivo no sistema de arquivos.
	fsInfo->writeFn = myFSWrite;

	// ssocia a função myFSClose ao campo closeFn da estrutura fsInfo. Esta função é utilizada para fechar um arquivo no sistema de arquivos.
	fsInfo->closeFn = myFSClose;

	// perdi um poquim da paciência

	// Adiciona a função de abrir diretório
	fsInfo->opendirFn = myFSOpenDir;

	// Adiciona a função de ler diretório
	fsInfo->readdirFn = myFSReadDir;

	// Adiciona a função de criar link
	fsInfo->linkFn = myFSLink;

	// Adiciona a função de remover link
	fsInfo->unlinkFn = myFSUnlink;

	// Adiciona a função de fechar diretório
	fsInfo->closedirFn = myFSCloseDir;
}
