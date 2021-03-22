#.rst:
# ConfigureFileWithCMakeIf
# ------------------------
#
# An enhanced version of configure_file that supports also template-style ifs.
#
#
# .. command:: configure_file_with_cmakeif
#
# Enhanced version of configure_file that permits to include or not several lines
# of the input document using the @cmakeif VARIABLE/@endcmakeif VARIABLE command
#
#  configure_file_with_cmakeif(<input> <output> ARGN)
#
# All the commands are shelled to the configure_file command called inside
#

#=============================================================================
# Copyright 2013 Istituto Italiano di Tecnologia (IIT)
#   Authors: Daniele E. Domenichelli <daniele.domenichelli@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)


if(COMMAND configure_file_with_cmakeif)
  return()
endif()


function(CONFIGURE_FILE_WITH_CMAKEIF _input_file _output)
  # Read input file
  file(READ ${_input_file} _input_string)

  if(_input_string STREQUAL "")
    message(FATAL_ERROR "Empty file ${_input_file}")
  endif()

  # Find couples of @cmakeif / @endcmakeif
  string(REGEX MATCHALL "@cmakeif[ ]+[A-Za-z0-9_]+" _matched_ifs "${_input_string}")
  list(LENGTH _matched_ifs _nr_of_matched_ifs)
  string(REGEX MATCHALL "@endcmakeif" _matched_endifs "${_input_string}")
  list(LENGTH _matched_endifs _nr_of_matched_endifs)

  if(NOT (${_nr_of_matched_ifs} EQUAL ${_nr_of_matched_endifs}))
    message(FATAL_ERROR "@cmakeif/@endcmakeif mismatch in file ${_input_file}")
  endif()

  foreach(_if ${_matched_ifs})
    string(REGEX MATCH "([A-Za-z0-9_]+)$" _condition ${_if})
    set(_condition ${CMAKE_MATCH_0})
    if(${_condition})
      # if the condition is valid, just strip the @cmakeif/@endcmakeif
      string(REPLACE ${_if} "" _input_string "${_input_string}")
      string(REPLACE "@endcmakeif ${_condition}" "" _input_string "${_input_string}")
    else()
      # if the condition is not valid, remove all the contents of the @cmakeif/@endcmakeif
      string(FIND "${_input_string}" "@cmakeif" _first_char_to_delete)
      string(FIND "${_input_string}" "@endcmakeif" _last_char_to_delete)
      string(LENGTH "@endcmakeif ${_condition}" _length_of_ending_tag)
      math(EXPR length_of_string_to_delete "${_last_char_to_delete}+${_length_of_ending_tag}-${_first_char_to_delete}")
      string(SUBSTRING "${_input_string}" ${_first_char_to_delete} ${length_of_string_to_delete} string_to_delete)
      string(REPLACE "${string_to_delete}" "" _input_string "${_input_string}")
    endif()
  endforeach()

  set(_processed_file ${CMAKE_CURRENT_BINARY_DIR}/temp-configure-file-with-cmakeif.txt)
  file(WRITE ${_processed_file} "${_input_string}")
  configure_file(${_processed_file} ${_output} ${ARGN})
endfunction()
