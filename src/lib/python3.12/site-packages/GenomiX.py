#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# @package GenomiX
# @author Florian Charriat

"""
	The module_Flo module
	=====================

	:author: CHARRIAT Florian\n
	:contact: florian.charriat@cirad.fr\n
	:date: 07/10/2025\n
	:version: 1.0.1\n

	Use it to import very handy functions.

	Example:

	>>> from GenomiX import fasta2dict
	>>> fasta2dict('/path/to/fasta/files/')
	
"""

##################################################
## Modules
##################################################
## Python modules
import argparse, os
## BioPython
from Bio import SeqIO
from Bio.SeqRecord import SeqRecord
from Bio.Seq import Seq


def fasta2dict(filename):
	"""
	Function that take a file name path (fasta), and return a dictionnary of sequence

	"""
	with open(filename, "rU") as fastaFile:
		return SeqIO.to_dict(SeqIO.parse(fastaFile, "fasta"))